#include "XboxCarControl.h"

#include <stdio.h>

namespace xboxcar {
namespace {

int32_t abs32(int32_t value) {
  // All callers pass values far inside INT32_MIN, so this remains defined.
  return value < 0 ? -value : value;
}

int16_t clampOutput(int32_t value) {
  if (value > kOutputLimit) {
    return kOutputLimit;
  }
  if (value < -kOutputLimit) {
    return -kOutputLimit;
  }
  return static_cast<int16_t>(value);
}

int16_t moveToward(int16_t current, int16_t target, int16_t step) {
  const int32_t difference =
      static_cast<int32_t>(target) - static_cast<int32_t>(current);
  if (difference > step) {
    return static_cast<int16_t>(static_cast<int32_t>(current) + step);
  }
  if (difference < -step) {
    return static_cast<int16_t>(static_cast<int32_t>(current) - step);
  }
  return target;
}

}  // namespace

bool DriveMixer::normalizeAxis(int32_t raw,
                               bool invert,
                               uint16_t deadzonePermille,
                               int16_t& normalized) {
  normalized = 0;
  if (raw < kXboxAxisMin || raw > kXboxAxisMax ||
      deadzonePermille >= 1000U) {
    return false;
  }

  int32_t value = invert ? -raw : raw;
  if (value > kXboxAxisMax) {
    value = kXboxAxisMax;
  } else if (value < kXboxAxisMin) {
    value = kXboxAxisMin;
  }

  const int32_t magnitude = abs32(value);
  const int32_t deadzone =
      (kXboxAxisMax * static_cast<int32_t>(deadzonePermille) + 500) / 1000;
  if (magnitude <= deadzone) {
    normalized = 0;
    return true;
  }

  // Bluepad32 reports the negative endpoint as -511 on Xbox controllers and
  // the positive endpoint as +512. Treat magnitude 511 as full scale so both
  // physical directions can reach exactly 1000.
  constexpr int32_t kNominalFullScaleMagnitude = 511;
  const int32_t usableRange = kNominalFullScaleMagnitude - deadzone;
  int32_t mapped =
      ((magnitude - deadzone) * kOutputLimit + usableRange / 2) / usableRange;
  if (mapped > kOutputLimit) {
    mapped = kOutputLimit;
  }
  normalized = static_cast<int16_t>(value < 0 ? -mapped : mapped);
  return true;
}

DriveMixResult DriveMixer::mix(int32_t rawLeftY, int32_t rawRightX) {
  DriveMixResult result;
  if (!normalizeAxis(rawLeftY, true, kLeftYDeadzonePermille,
                     result.target.throttle) ||
      !normalizeAxis(rawRightX, false, kRightXDeadzonePermille,
                     result.target.steering)) {
    result.valid = false;
    result.target = DriveTarget{};
    result.target.command = DriveCommand::Error;
    return result;
  }

  int32_t steeringForMix = result.target.steering;
  if (result.target.throttle == 0) {
    steeringForMix =
        steeringForMix * static_cast<int32_t>(kSpinOutputLimit) / kOutputLimit;
  } else {
    steeringForMix =
        steeringForMix * static_cast<int32_t>(kDriveSteeringGain) /
        kOutputLimit;
  }

  int32_t left = static_cast<int32_t>(result.target.throttle) + steeringForMix;
  int32_t right =
      static_cast<int32_t>(result.target.throttle) - steeringForMix;

  int32_t scale = kOutputLimit;
  const int32_t leftMagnitude = abs32(left);
  const int32_t rightMagnitude = abs32(right);
  if (leftMagnitude > scale) {
    scale = leftMagnitude;
  }
  if (rightMagnitude > scale) {
    scale = rightMagnitude;
  }

  if (scale > kOutputLimit) {
    left = left * kOutputLimit / scale;
    right = right * kOutputLimit / scale;
  }

  result.target.left = clampOutput(left);
  result.target.right = clampOutput(right);
  return result;
}

void DriveRamp::reset(int16_t left, int16_t right) {
  currentLeft_ = clampOutput(left);
  currentRight_ = clampOutput(right);
}

int16_t DriveRamp::stepAxis(int16_t current, int16_t target) {
  target = clampOutput(target);
  const int32_t current32 = current;
  const int32_t target32 = target;

  // Never cross directly from forward to reverse. Decelerate to zero first.
  if ((current32 > 0 && target32 < 0) ||
      (current32 < 0 && target32 > 0)) {
    return moveToward(current, 0, kRampDecelerationStep);
  }

  const int32_t currentMagnitude = abs32(current32);
  const int32_t targetMagnitude = abs32(target32);
  const int16_t step = targetMagnitude > currentMagnitude
                           ? kRampAccelerationStep
                           : kRampDecelerationStep;
  return moveToward(current, target, step);
}

void DriveRamp::update(int16_t targetLeft, int16_t targetRight) {
  currentLeft_ = stepAxis(currentLeft_, targetLeft);
  currentRight_ = stepAxis(currentRight_, targetRight);
}

DriveCommand CommandClassifier::classify(bool connected,
                                         bool hasSample,
                                         bool emergencyStop,
                                         bool controlError,
                                         int16_t throttle,
                                         int16_t steering,
                                         int16_t left,
                                         int16_t right) {
  if (controlError) {
    return DriveCommand::Error;
  }
  if (emergencyStop) {
    return DriveCommand::EmergencyStop;
  }
  if (!connected) {
    return DriveCommand::Disconnected;
  }
  if (!hasSample) {
    return DriveCommand::NoInput;
  }
  if (left == 0 && right == 0) {
    return DriveCommand::Stop;
  }

  if (left < 0 && right > 0) {
    return DriveCommand::SpinLeft;
  }
  if (left > 0 && right < 0) {
    return DriveCommand::SpinRight;
  }

  // While driving, direction of the normalized steering input is authoritative.
  if (throttle != 0 && steering < 0) {
    return DriveCommand::TurnLeft;
  }
  if (throttle != 0 && steering > 0) {
    return DriveCommand::TurnRight;
  }
  if (left > 0 && right > 0) {
    return DriveCommand::Forward;
  }
  if (left < 0 && right < 0) {
    return DriveCommand::Reverse;
  }

  return DriveCommand::Stop;
}

uint8_t Stm32Protocol::xorCrc(const char* data, size_t length) {
  uint8_t crc = 0;
  if (data == nullptr) {
    return crc;
  }
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint8_t>(data[i]);
  }
  return crc;
}

bool Stm32Protocol::commandBits(DriveCommand command, char output[5]) {
  if (output == nullptr) {
    return false;
  }

  const uint16_t value = static_cast<uint16_t>(command);
  if (value > 0x0fU) {
    output[0] = '\0';
    return false;
  }

  for (int bit = 3; bit >= 0; --bit) {
    output[3 - bit] = (value & (1U << bit)) != 0U ? '1' : '0';
  }
  output[4] = '\0';
  return true;
}

bool Stm32Protocol::buildFrame(DriveCommand command,
                               int16_t left,
                               int16_t right,
                               char* output,
                               size_t outputCapacity,
                               uint16_t* usedSequence) {
  if (output == nullptr || outputCapacity < kFrameBufferSize ||
      left < -kOutputLimit || left > kOutputLimit ||
      right < -kOutputLimit || right > kOutputLimit) {
    return false;
  }

  char commandText[5];
  if (!commandBits(command, commandText)) {
    return false;
  }

  char payload[29];
  const int payloadLength =
      snprintf(payload, sizeof(payload), "XC,%s,%+05d,%+05d,%04u",
               commandText, static_cast<int>(left), static_cast<int>(right),
               static_cast<unsigned>(sequence_));
  if (payloadLength <= 0 ||
      static_cast<size_t>(payloadLength) >= sizeof(payload)) {
    return false;
  }

  const uint8_t crc = xorCrc(payload, static_cast<size_t>(payloadLength));
  const int frameLength =
      snprintf(output, outputCapacity, "$%s,%02X\r\n", payload, crc);
  if (frameLength <= 0 ||
      static_cast<size_t>(frameLength) >= outputCapacity) {
    output[0] = '\0';
    return false;
  }

  if (usedSequence != nullptr) {
    *usedSequence = sequence_;
  }
  sequence_ = static_cast<uint16_t>((sequence_ + 1U) % 10000U);
  return true;
}

bool Stm32Protocol::buildFrameV2(DriveCommand command,
                                 int16_t left,
                                 int16_t right,
                                 int16_t upDown,
                                 int16_t leftRight,
                                 int16_t rawUpDown,
                                 int16_t rawLeftRight,
                                 uint8_t flags,
                                 char* output,
                                 size_t outputCapacity,
                                 uint16_t* usedSequence) {
  if (output == nullptr || outputCapacity < kFrameBufferSize ||
      left < -kOutputLimit || left > kOutputLimit ||
      right < -kOutputLimit || right > kOutputLimit ||
      upDown < -kOutputLimit || upDown > kOutputLimit ||
      leftRight < -kOutputLimit || leftRight > kOutputLimit ||
      rawUpDown < -9999 || rawUpDown > 9999 ||
      rawLeftRight < -9999 || rawLeftRight > 9999) {
    return false;
  }

  char commandText[5];
  if (!commandBits(command, commandText)) {
    return false;
  }

  char payload[53];
  const int payloadLength =
      snprintf(payload, sizeof(payload),
               "XD,%s,%+05d,%+05d,%+05d,%+05d,%+05d,%+05d,%02X,%04u",
               commandText, static_cast<int>(left), static_cast<int>(right),
               static_cast<int>(upDown), static_cast<int>(leftRight),
               static_cast<int>(rawUpDown), static_cast<int>(rawLeftRight),
               static_cast<unsigned>(flags), static_cast<unsigned>(sequence_));
  if (payloadLength <= 0 || static_cast<size_t>(payloadLength) >= sizeof(payload)) {
    return false;
  }

  const uint8_t crc = xorCrc(payload, static_cast<size_t>(payloadLength));
  const int frameLength =
      snprintf(output, outputCapacity, "$%s,%02X\r\n", payload, crc);
  if (frameLength <= 0 || static_cast<size_t>(frameLength) >= outputCapacity) {
    output[0] = '\0';
    return false;
  }

  if (usedSequence != nullptr) {
    *usedSequence = sequence_;
  }
  sequence_ = static_cast<uint16_t>((sequence_ + 1U) % 10000U);
  return true;
}

}  // namespace xboxcar
