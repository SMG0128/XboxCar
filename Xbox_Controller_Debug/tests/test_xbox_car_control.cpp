#include "../XboxCarControl.h"

#include <assert.h>
#include <string.h>

#include <iostream>

using namespace xboxcar;

namespace {

void testMixerCardinalDirections() {
  const DriveMixResult stop = DriveMixer::mix(0, 0);
  assert(stop.valid);
  assert(stop.target.throttle == 0);
  assert(stop.target.steering == 0);
  assert(stop.target.left == 0);
  assert(stop.target.right == 0);

  const DriveMixResult forward = DriveMixer::mix(-511, 0);
  assert(forward.valid);
  assert(forward.target.throttle == 1000);
  assert(forward.target.left == 1000);
  assert(forward.target.right == 1000);

  const DriveMixResult reverse = DriveMixer::mix(512, 0);
  assert(reverse.valid);
  assert(reverse.target.throttle == -1000);
  assert(reverse.target.left == -1000);
  assert(reverse.target.right == -1000);

  const DriveMixResult spinRight = DriveMixer::mix(0, 512);
  assert(spinRight.valid);
  assert(spinRight.target.left == 650);
  assert(spinRight.target.right == -650);

  const DriveMixResult spinLeft = DriveMixer::mix(0, -511);
  assert(spinLeft.valid);
  assert(spinLeft.target.left == -650);
  assert(spinLeft.target.right == 650);
}

void testMixerTurningAndLimits() {
  const DriveMixResult right = DriveMixer::mix(-400, 300);
  assert(right.valid);
  assert(right.target.left > right.target.right);

  const DriveMixResult left = DriveMixer::mix(-400, -300);
  assert(left.valid);
  assert(left.target.right > left.target.left);

  const DriveMixResult saturated = DriveMixer::mix(-512, 512);
  assert(saturated.valid);
  assert(saturated.target.left <= 1000);
  assert(saturated.target.left >= -1000);
  assert(saturated.target.right <= 1000);
  assert(saturated.target.right >= -1000);

  const DriveMixResult invalid = DriveMixer::mix(-32768, 0);
  assert(!invalid.valid);
  assert(invalid.target.left == 0);
  assert(invalid.target.right == 0);
  assert(invalid.target.command == DriveCommand::Error);
}

void testLinearDeadzoneRemap() {
  int16_t atBoundary = -1;
  int16_t justOutside = -1;
  assert(DriveMixer::normalizeAxis(-41, true, kLeftYDeadzonePermille,
                                   atBoundary));
  assert(DriveMixer::normalizeAxis(-42, true, kLeftYDeadzonePermille,
                                   justOutside));
  assert(atBoundary == 0);
  assert(justOutside > 0);
  assert(justOutside <= 5);

  int16_t rightBoundary = -1;
  int16_t rightOutside = -1;
  assert(DriveMixer::normalizeAxis(51, false, kRightXDeadzonePermille,
                                   rightBoundary));
  assert(DriveMixer::normalizeAxis(52, false, kRightXDeadzonePermille,
                                   rightOutside));
  assert(rightBoundary == 0);
  assert(rightOutside > 0);
  assert(rightOutside <= 5);
}

void testRampAndDirectionReversal() {
  DriveRamp ramp;
  ramp.update(1000, 1000);
  assert(ramp.left() == 25);
  assert(ramp.right() == 25);

  for (int i = 1; i < 40; ++i) {
    ramp.update(1000, 1000);
  }
  assert(ramp.left() == 1000);
  assert(ramp.right() == 1000);

  ramp.update(0, 0);
  assert(ramp.left() == 960);
  assert(ramp.right() == 960);

  ramp.reset(100, 100);
  ramp.update(-1000, -1000);
  assert(ramp.left() == 60);
  assert(ramp.right() == 60);
  ramp.update(-1000, -1000);
  assert(ramp.left() == 20);
  ramp.update(-1000, -1000);
  assert(ramp.left() == 0);
  ramp.update(-1000, -1000);
  assert(ramp.left() == -25);
}

void testCommandsAndDisconnectSafety() {
  assert(CommandClassifier::classify(true, true, false, false, 0, 0, 0,
                                     0) == DriveCommand::Stop);
  assert(CommandClassifier::classify(true, true, false, false, 1000, 0,
                                     500, 500) == DriveCommand::Forward);
  assert(CommandClassifier::classify(true, true, false, false, -1000, 0,
                                     -500, -500) == DriveCommand::Reverse);
  assert(CommandClassifier::classify(true, true, false, false, 500, -300,
                                     300, 700) == DriveCommand::TurnLeft);
  assert(CommandClassifier::classify(true, true, false, false, 500, 300,
                                     700, 300) == DriveCommand::TurnRight);
  assert(CommandClassifier::classify(true, true, false, false, 0, -1000,
                                     -650, 650) == DriveCommand::SpinLeft);
  assert(CommandClassifier::classify(true, true, false, false, 0, 1000,
                                     650, -650) == DriveCommand::SpinRight);
  assert(CommandClassifier::classify(false, false, false, false, 0, 0, 0,
                                     0) == DriveCommand::Disconnected);
  assert(CommandClassifier::classify(true, true, true, false, 0, 0, 0,
                                     0) == DriveCommand::EmergencyStop);

  Stm32Protocol protocol;
  char frame[Stm32Protocol::kFrameBufferSize];
  assert(protocol.buildFrame(DriveCommand::Disconnected, 0, 0, frame,
                             sizeof(frame)));
  assert(strcmp(frame, "$XC,1001,+0000,+0000,0000,1B\r\n") == 0);
  assert(strlen(frame) == 30);
}

void testProtocolCrcAndSequenceWrap() {
  const char payload[] = "XC,0001,+0800,+0800,0025";
  assert(Stm32Protocol::xorCrc(payload, strlen(payload)) == 0x1d);

  Stm32Protocol protocol;
  protocol.setNextSequence(9999);
  char frame[Stm32Protocol::kFrameBufferSize];
  uint16_t usedSequence = 0;
  assert(protocol.buildFrame(DriveCommand::Stop, 0, 0, frame, sizeof(frame),
                             &usedSequence));
  assert(usedSequence == 9999);
  assert(protocol.nextSequence() == 0);

  assert(protocol.buildFrame(DriveCommand::Forward, 800, 800, frame,
                             sizeof(frame), &usedSequence));
  assert(usedSequence == 0);
  const char expectedPrefix[] = "$XC,0001,+0800,+0800,0000,";
  assert(strncmp(frame, expectedPrefix, strlen(expectedPrefix)) == 0);

  char tooSmall[8];
  assert(!protocol.buildFrame(DriveCommand::Stop, 0, 0, tooSmall,
                              sizeof(tooSmall)));
  assert(!protocol.buildFrame(DriveCommand::Stop, 1001, 0, frame,
                              sizeof(frame)));
}

}  // namespace

int main() {
  testMixerCardinalDirections();
  testMixerTurningAndLimits();
  testLinearDeadzoneRemap();
  testRampAndDirectionReversal();
  testCommandsAndDisconnectSafety();
  testProtocolCrcAndSequenceWrap();

  std::cout << "All XboxCar control tests passed.\n";
  return 0;
}
