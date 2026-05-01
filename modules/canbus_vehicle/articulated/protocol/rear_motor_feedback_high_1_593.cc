#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_high_1_593.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t RearMotorFeedbackHigh1_593::ID = 0x251;

void RearMotorFeedbackHigh1_593::Parse(const std::uint8_t* bytes,
                                        int32_t length,
                                        Articulated* chassis) const {
  chassis->mutable_rear_motor_feedback_high_1_593()->set_motor1_speed(
      motor1_speed(bytes, length));
  chassis->mutable_rear_motor_feedback_high_1_593()->set_motor1_current(
      motor1_current(bytes, length));
}

// bit:7, signed, len:16, motorola, factor:1.0, unit:RPM
int32_t RearMotorFeedbackHigh1_593::motor1_speed(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x;
}

// bit:23, signed, len:16, motorola, factor:0.1, unit:A
double RearMotorFeedbackHigh1_593::motor1_current(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
