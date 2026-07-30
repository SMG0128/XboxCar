#include <Bluepad32.h>

#include "XboxCarControl.h"

// 0: protocol frames only; 1: connection/safety events; 2: rate-limited control diagnostics.
#ifndef XBOXCAR_LOG_LEVEL
#define XBOXCAR_LOG_LEVEL 0
#endif

namespace {

using xboxcar::CommandClassifier;
using xboxcar::DriveCommand;
using xboxcar::DriveMixResult;
using xboxcar::DriveMixer;
using xboxcar::DriveRamp;
using xboxcar::DriveTarget;
using xboxcar::Stm32Protocol;
using xboxcar::XboxDriveInput;

constexpr uint32_t kDiagnosticPeriodMs = 250;
constexpr uint8_t kMaxControlCatchUpSteps = 5;

ControllerPtr controllers[BP32_MAX_GAMEPADS];
ControllerPtr primaryController;

XboxDriveInput xboxInput;
DriveMixResult mixResult;
DriveRamp driveRamp;
Stm32Protocol protocol;

bool uartConfigured = false;
bool emergencyStop = false;
bool controlError = false;

uint32_t lastControlMs = 0;
uint32_t lastTransmitMs = 0;
uint32_t lastDiagnosticMs = 0;

#define LOG_EVENT(...)                                      \
  do {                                                      \
    if (XBOXCAR_LOG_LEVEL >= 1) Console.printf(__VA_ARGS__); \
  } while (0)

#define LOG_DIAGNOSTIC(...)                                 \
  do {                                                      \
    if (XBOXCAR_LOG_LEVEL >= 2) Console.printf(__VA_ARGS__); \
  } while (0)

bool sendFrame(DriveCommand command, int16_t left, int16_t right) {
  if (!uartConfigured) {
    return false;
  }

  char frame[Stm32Protocol::kFrameBufferSize];
  if (!protocol.buildFrame(command, left, right, frame, sizeof(frame))) {
    controlError = true;
    return false;
  }

  // Bluepad32 Console expands '\n' to CRLF. Remove the frame's explicit CR
  // before handing it to Console so the bytes on UART remain exactly CRLF.
  const size_t frameLength = strlen(frame);
  if (frameLength < 2 || frame[frameLength - 2] != '\r' ||
      frame[frameLength - 1] != '\n') {
    controlError = true;
    return false;
  }
  frame[frameLength - 2] = '\n';
  frame[frameLength - 1] = '\0';

  // The complete transport frame is still submitted in one API call.
  Console.print(frame);
  return true;
}

void forceZeroOutput(DriveCommand command) {
  driveRamp.reset();
  mixResult = DriveMixResult{};
  mixResult.target.command = command;
  sendFrame(command, 0, 0);
}

void onConnectedController(ControllerPtr controller) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllers[i] == nullptr) {
      controllers[i] = controller;
      if (primaryController == nullptr) {
        primaryController = controller;
        xboxInput = XboxDriveInput{};
        xboxInput.connected = true;
        emergencyStop = false;
        controlError = false;
        forceZeroOutput(DriveCommand::NoInput);
      }

      const ControllerProperties properties = controller->getProperties();
      LOG_EVENT("[EVENT] Xbox connected: slot=%d model=%s VID=%04x PID=%04x\n",
                i, controller->getModelName().c_str(), properties.vendor_id,
                properties.product_id);
      return;
    }
  }

  LOG_EVENT("[EVENT] Controller ignored: no free slot\n");
}

void onDisconnectedController(ControllerPtr controller) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllers[i] == controller) {
      controllers[i] = nullptr;
      break;
    }
  }

  if (primaryController == controller) {
    primaryController = nullptr;
    xboxInput = XboxDriveInput{};
    emergencyStop = false;
    controlError = false;

    // A disconnect bypasses the normal ramp: targets and transmitted outputs
    // become zero immediately.
    forceZeroOutput(DriveCommand::Disconnected);
    LOG_EVENT("[EVENT] Xbox disconnected: immediate safe stop\n");
  }
}

void samplePrimaryController() {
  ControllerPtr controller = primaryController;
  if (controller == nullptr || !controller->isConnected()) {
    return;
  }

  xboxInput.connected = true;
  if (!controller->hasData() || !controller->isGamepad()) {
    return;
  }

  // Bluepad32 actual fields:
  // axisY()  = left stick Y (forward is negative, inverted by DriveMixer)
  // axisRX() = right stick X (right is positive)
  xboxInput.leftY = controller->axisY();
  xboxInput.rightX = controller->axisRX();
  xboxInput.hasSample = true;

  const bool newEmergencyStop =
      (controller->miscButtons() & MISC_BUTTON_SYSTEM) != 0;
  if (newEmergencyStop && !emergencyStop) {
    emergencyStop = true;
    forceZeroOutput(DriveCommand::EmergencyStop);
    LOG_EVENT("[SAFETY] Xbox system button: emergency stop\n");
  } else if (!newEmergencyStop) {
    emergencyStop = false;
  }
}

DriveCommand currentCommand() {
  return CommandClassifier::classify(
      xboxInput.connected, xboxInput.hasSample, emergencyStop, controlError,
      mixResult.target.throttle, mixResult.target.steering, driveRamp.left(),
      driveRamp.right());
}

void updateControl() {
  if (!xboxInput.connected || !xboxInput.hasSample || emergencyStop ||
      controlError) {
    driveRamp.reset();
    mixResult = DriveMixResult{};
    return;
  }

  mixResult = DriveMixer::mix(xboxInput.leftY, xboxInput.rightX);
  if (!mixResult.valid) {
    controlError = true;
    forceZeroOutput(DriveCommand::Error);
    LOG_EVENT("[SAFETY] Invalid Xbox axis data: emergency zero output\n");
    return;
  }

  driveRamp.update(mixResult.target.left, mixResult.target.right);
  mixResult.target.command = currentCommand();
}

void transmitPeriodicFrame() {
  const DriveCommand command = currentCommand();
  if (command == DriveCommand::Disconnected ||
      command == DriveCommand::NoInput ||
      command == DriveCommand::EmergencyStop ||
      command == DriveCommand::Error) {
    sendFrame(command, 0, 0);
  } else {
    sendFrame(command, driveRamp.left(), driveRamp.right());
  }
}

void printRateLimitedDiagnostic(uint32_t now) {
  if (XBOXCAR_LOG_LEVEL < 2 ||
      static_cast<uint32_t>(now - lastDiagnosticMs) < kDiagnosticPeriodMs) {
    return;
  }
  lastDiagnosticMs = now;

  char commandText[5] = "1111";
  Stm32Protocol::commandBits(currentCommand(), commandText);
  LOG_DIAGNOSTIC(
      "[CTRL] raw_leftY=%ld raw_rightX=%ld throttle=%d steering=%d "
      "target_left=%d target_right=%d current_left=%d current_right=%d "
      "CMD=%s\n",
      static_cast<long>(xboxInput.leftY),
      static_cast<long>(xboxInput.rightX), mixResult.target.throttle,
      mixResult.target.steering, mixResult.target.left, mixResult.target.right,
      driveRamp.left(), driveRamp.right(), commandText);
}

}  // namespace

void setup() {
  // Reuse the already-proven Bluepad32 UART0 console configuration. For the
  // generic ESP32-S3 variant this is 115200 baud, TX GPIO43, RX GPIO44, 8N1.
  Console.begin(115200);
  uartConfigured = true;

  // Safe boot default: publish zero output before accepting controller input.
  forceZeroOutput(DriveCommand::Stop);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  lastControlMs = millis();
  lastTransmitMs = lastControlMs;
  lastDiagnosticMs = lastControlMs;
}

void loop() {
  if (BP32.update()) {
    samplePrimaryController();
  }

  const uint32_t now = millis();
  uint8_t catchUpSteps = 0;
  while (static_cast<uint32_t>(now - lastControlMs) >=
             xboxcar::kControlPeriodMs &&
         catchUpSteps < kMaxControlCatchUpSteps) {
    lastControlMs += xboxcar::kControlPeriodMs;
    updateControl();
    ++catchUpSteps;
  }
  if (catchUpSteps == kMaxControlCatchUpSteps &&
      static_cast<uint32_t>(now - lastControlMs) >=
          xboxcar::kControlPeriodMs) {
    lastControlMs = now;
  }

  if (static_cast<uint32_t>(now - lastTransmitMs) >=
      xboxcar::kTransmitPeriodMs) {
    lastTransmitMs = now;
    transmitPeriodicFrame();
  }

  printRateLimitedDiagnostic(now);
  delay(1);
}
