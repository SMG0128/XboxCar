#pragma once

#include <stddef.h>
#include <stdint.h>

namespace xboxcar {

constexpr int16_t kOutputLimit = 1000;
constexpr int16_t kSpinOutputLimit = 650;
constexpr int16_t kDriveSteeringGain = 850;
constexpr int16_t kRampAccelerationStep = 25;
constexpr int16_t kRampDecelerationStep = 40;
constexpr uint16_t kControlPeriodMs = 10;
constexpr uint16_t kTransmitPeriodMs = 20;
constexpr uint16_t kLeftYDeadzonePermille = 80;
constexpr uint16_t kRightXDeadzonePermille = 100;
constexpr int32_t kXboxAxisMin = -512;
constexpr int32_t kXboxAxisMax = 512;

enum class DriveCommand : uint16_t {
  Stop = 0b0000,
  Forward = 0b0001,
  Reverse = 0b0010,
  TurnLeft = 0b0011,
  TurnRight = 0b0100,
  SpinLeft = 0b0101,
  SpinRight = 0b0110,
  EmergencyStop = 0b0111,
  NoInput = 0b1000,
  Disconnected = 0b1001,
  Error = 0b1111,
};

struct XboxDriveInput {
  int32_t leftY = 0;
  int32_t rightX = 0;
  bool connected = false;
  bool hasSample = false;
};

struct DriveTarget {
  int16_t throttle = 0;
  int16_t steering = 0;
  int16_t left = 0;
  int16_t right = 0;
  DriveCommand command = DriveCommand::Stop;
};

struct DriveMixResult {
  DriveTarget target{};
  bool valid = true;
};

class DriveMixer {
 public:
  static DriveMixResult mix(int32_t rawLeftY, int32_t rawRightX);

  // Exposed for deterministic host-side tests.
  static bool normalizeAxis(int32_t raw,
                            bool invert,
                            uint16_t deadzonePermille,
                            int16_t& normalized);
};

class DriveRamp {
 public:
  void reset(int16_t left = 0, int16_t right = 0);
  void update(int16_t targetLeft, int16_t targetRight);

  int16_t left() const { return currentLeft_; }
  int16_t right() const { return currentRight_; }

 private:
  static int16_t stepAxis(int16_t current, int16_t target);

  int16_t currentLeft_ = 0;
  int16_t currentRight_ = 0;
};

class CommandClassifier {
 public:
  static DriveCommand classify(bool connected,
                               bool hasSample,
                               bool emergencyStop,
                               bool controlError,
                               int16_t throttle,
                               int16_t steering,
                               int16_t left,
                               int16_t right);
};

class Stm32Protocol {
 public:
  /* 57-byte v2 frame plus the NUL terminator; v1 also fits in this buffer. */
  static constexpr size_t kFrameBufferSize = 64;

  bool buildFrame(DriveCommand command,
                  int16_t left,
                  int16_t right,
                  char* output,
                  size_t outputCapacity,
                  uint16_t* usedSequence = nullptr);

  bool buildFrameV2(DriveCommand command,
                    int16_t left,
                    int16_t right,
                    int16_t upDown,
                    int16_t leftRight,
                    int16_t rawUpDown,
                    int16_t rawLeftRight,
                    uint8_t flags,
                    char* output,
                    size_t outputCapacity,
                    uint16_t* usedSequence = nullptr);

  uint16_t nextSequence() const { return sequence_; }
  void setNextSequence(uint16_t sequence) {
    sequence_ = static_cast<uint16_t>(sequence % 10000U);
  }

  static uint8_t xorCrc(const char* data, size_t length);
  static bool commandBits(DriveCommand command, char output[5]);

 private:
  uint16_t sequence_ = 0;
};

}  // namespace xboxcar
