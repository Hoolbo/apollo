#include "modules/canbus_vehicle/articulated/protocol/front_drivemotor_acu_572.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t FrontDrivemotorAcu572::ID = 0x23C;

void FrontDrivemotorAcu572::Parse(const std::uint8_t* bytes, int32_t length,
                                  Articulated* chassis) const {
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_speed(
      drive_motor_speed(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_torque(
      drive_motor_torque(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_shift(
      drive_motor_shift(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_enable(
      drive_motor_enable(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_reply(
      drive_motor_reply(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_mode(
      drive_motor_mode(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_lock(
      drive_motor_lock(bytes, length));
  chassis->mutable_front_drivemotor_acu_572()->set_drive_motor_stop(
      drive_motor_stop(bytes, length));
}

double FrontDrivemotorAcu572::drive_motor_speed(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte frame(bytes + 5);
  int32_t low = frame.get_byte(0, 8);
  Byte frame_high(bytes + 6);
  int32_t high = frame_high.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.1;
}

double FrontDrivemotorAcu572::drive_motor_torque(const std::uint8_t* bytes,
                                                 int32_t length) const {
  Byte frame(bytes + 3);
  int32_t low = frame.get_byte(0, 8);
  Byte frame_high(bytes + 4);
  int32_t high = frame_high.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 1.0;
}

ArticulatedFrontDrivemotorAcu572::ShiftType
FrontDrivemotorAcu572::drive_motor_shift(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte frame(bytes + 7);
  int32_t x = frame.get_byte(4, 2);
  ArticulatedFrontDrivemotorAcu572::ShiftType ret =
      ArticulatedFrontDrivemotorAcu572::SHIFT_P;
  switch (x) {
    case 0:
      ret = ArticulatedFrontDrivemotorAcu572::SHIFT_P;
      break;
    case 1:
      ret = ArticulatedFrontDrivemotorAcu572::SHIFT_D;
      break;
    case 2:
      ret = ArticulatedFrontDrivemotorAcu572::SHIFT_R;
      break;
    case 3:
      ret = ArticulatedFrontDrivemotorAcu572::SHIFT_N;
      break;
    default:
      ret = ArticulatedFrontDrivemotorAcu572::SHIFT_P;
      break;
  }
  return ret;
}

bool FrontDrivemotorAcu572::drive_motor_enable(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

bool FrontDrivemotorAcu572::drive_motor_reply(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(3);
}

bool FrontDrivemotorAcu572::drive_motor_mode(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(1);
}

bool FrontDrivemotorAcu572::drive_motor_lock(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(2);
}

bool FrontDrivemotorAcu572::drive_motor_stop(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte frame(bytes + 0);
  return frame.is_bit_1(0);
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
