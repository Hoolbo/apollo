#include "modules/canbus_vehicle/mycar/protocol/drivemotor_acu_572.h"

#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mycar {

using ::apollo::drivers::canbus::Byte;

const int32_t DrivemotorAcu572::ID = 0x23C;

void DrivemotorAcu572::Parse(const std::uint8_t* bytes, int32_t length,
                             Mycar* chassis) const {
  chassis->mutable_drivemotor_acu_572()->set_drive_motor_speed(
      drive_motor_speed(bytes, length));
  chassis->mutable_drivemotor_acu_572()->set_drive_motor_torque(
      drive_motor_torque(bytes, length));
  chassis->mutable_drivemotor_acu_572()->set_drive_motor_shift(
      drive_motor_shift(bytes, length));
  
  // Parse other flags...
  chassis->mutable_drivemotor_acu_572()->set_drive_motor_enable(
      drive_motor_enable(bytes, length));
}

double DrivemotorAcu572::drive_motor_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 5);
  int32_t low = frame.get_byte(0, 8);
  Byte frame_high(bytes + 6);
  int32_t high = frame_high.get_byte(0, 8);
  int32_t value = (high << 8) | low;
  return value * 0.1;
}

double DrivemotorAcu572::drive_motor_torque(const std::uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 3);
  int32_t low = frame.get_byte(0, 8);
  Byte frame_high(bytes + 4); // Assuming Intel standard usually spans upward
  // DBC said 24|16. StartBit 24 is Byte 3.
  int32_t high = frame_high.get_byte(0, 8);
  int32_t value = (high << 8) | low; 
  return value * 1.0; 
}

MycarDrivemotorAcu572::ShiftType DrivemotorAcu572::drive_motor_shift(const std::uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 7);
  int32_t x = frame.get_byte(4, 2);
  MycarDrivemotorAcu572::ShiftType ret = MycarDrivemotorAcu572::SHIFT_P;
  switch (x) {
    case 0: ret = MycarDrivemotorAcu572::SHIFT_P; break;
    case 1: ret = MycarDrivemotorAcu572::SHIFT_D; break;
    case 2: ret = MycarDrivemotorAcu572::SHIFT_R; break;
    case 3: ret = MycarDrivemotorAcu572::SHIFT_N; break;
    default: ret = MycarDrivemotorAcu572::SHIFT_P; break;
  }
  return ret;
}

bool DrivemotorAcu572::drive_motor_enable(const std::uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 7);
  return frame.is_bit_1(0);
}

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
