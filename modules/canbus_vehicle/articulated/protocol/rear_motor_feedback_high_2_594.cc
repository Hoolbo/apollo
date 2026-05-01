#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_high_2_594.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t RearMotorFeedbackHigh2_594::ID = 0x252;

void RearMotorFeedbackHigh2_594::Parse(const std::uint8_t* bytes,
                                        int32_t length,
                                        Articulated* chassis) const {
  chassis->mutable_rear_motor_feedback_high_2_594()->set_motor2_speed(
      motor2_speed(bytes, length));
  chassis->mutable_rear_motor_feedback_high_2_594()->set_motor2_current(
      motor2_current(bytes, length));
}

int32_t RearMotorFeedbackHigh2_594::motor2_speed(const std::uint8_t* bytes,
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

double RearMotorFeedbackHigh2_594::motor2_current(const std::uint8_t* bytes,
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
