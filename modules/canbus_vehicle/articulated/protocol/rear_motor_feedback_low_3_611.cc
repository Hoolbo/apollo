#include "modules/canbus_vehicle/articulated/protocol/rear_motor_feedback_low_3_611.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t RearMotorFeedbackLow3_611::ID = 0x263;

void RearMotorFeedbackLow3_611::Parse(const std::uint8_t* bytes,
                                       int32_t length,
                                       Articulated* chassis) const {
  chassis->mutable_rear_motor_feedback_low_3_611()->set_driver_voltage_3(
      driver_voltage_3(bytes, length));
  chassis->mutable_rear_motor_feedback_low_3_611()->set_driver_temp_3(
      driver_temp_3(bytes, length));
  chassis->mutable_rear_motor_feedback_low_3_611()->set_motor_temp_3(
      motor_temp_3(bytes, length));
  chassis->mutable_rear_motor_feedback_low_3_611()->set_driver_status_3(
      driver_status_3(bytes, length));
}

double RearMotorFeedbackLow3_611::driver_voltage_3(const std::uint8_t* bytes,
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

int32_t RearMotorFeedbackLow3_611::driver_temp_3(const std::uint8_t* bytes,
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

int32_t RearMotorFeedbackLow3_611::motor_temp_3(const std::uint8_t* bytes,
                                                  int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  return x;
}

int32_t RearMotorFeedbackLow3_611::driver_status_3(const std::uint8_t* bytes,
                                                     int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  return x;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
