#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_high_3_595.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t RearMotorFeedbackHigh3_595::ID = 0x253;

void RearMotorFeedbackHigh3_595::Parse(const std::uint8_t* bytes,
                                        int32_t length,
                                        Articulated* chassis) const {
  chassis->mutable_rear_motor_feedback_high_3_595()->set_motor3_speed(
      motor3_speed(bytes, length));
  chassis->mutable_rear_motor_feedback_high_3_595()->set_motor3_current(
      motor3_current(bytes, length));
}

int32_t RearMotorFeedbackHigh3_595::motor3_speed(const std::uint8_t* bytes,
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

double RearMotorFeedbackHigh3_595::motor3_current(const std::uint8_t* bytes,
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
