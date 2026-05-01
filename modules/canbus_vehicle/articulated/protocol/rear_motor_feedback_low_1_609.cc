#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_low_1_609.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t RearMotorFeedbackLow1_609::ID = 0x261;

void RearMotorFeedbackLow1_609::Parse(const std::uint8_t* bytes,
                                       int32_t length,
                                       Articulated* chassis) const {
  chassis->mutable_rear_motor_feedback_low_1_609()->set_driver_voltage_1(
      driver_voltage_1(bytes, length));
  chassis->mutable_rear_motor_feedback_low_1_609()->set_driver_temp_1(
      driver_temp_1(bytes, length));
  chassis->mutable_rear_motor_feedback_low_1_609()->set_motor_temp_1(
      motor_temp_1(bytes, length));
  chassis->mutable_rear_motor_feedback_low_1_609()->set_driver_status_1(
      driver_status_1(bytes, length));
}

// bit:7, unsigned, len:16, motorola, factor:0.1, unit:V
double RearMotorFeedbackLow1_609::driver_voltage_1(const std::uint8_t* bytes,
                                                    int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// bit:23, signed, len:16, motorola, factor:1.0, unit:degC
int32_t RearMotorFeedbackLow1_609::driver_temp_1(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  return x;
}

// bit:39, signed, len:8, motorola, factor:1.0, unit:degC
int32_t RearMotorFeedbackLow1_609::motor_temp_1(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  return x;
}

// bit:47, unsigned, len:8, motorola, factor:1.0
int32_t RearMotorFeedbackLow1_609::driver_status_1(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
