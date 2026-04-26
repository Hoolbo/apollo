#include "modules/canbus_vehicle/articulated/protocol/rear_chassis_status_529.h"

#include "glog/logging.h"

namespace apollo {
namespace canbus {
namespace articulated {

const int32_t RearChassisStatus529::ID = 0x211;

void RearChassisStatus529::Parse(const std::uint8_t* bytes, int32_t length,
                                 Articulated* chassis) const {
  auto* status = chassis->mutable_rear_chassis_status_529();

  int32_t vs = vehicle_state(bytes, length);
  if (vs == 0)
    status->set_vehicle_state(ArticulatedRearChassisStatus529::NORMAL);
  else if (vs == 1)
    status->set_vehicle_state(ArticulatedRearChassisStatus529::EMERGENCY_STOP);
  else
    status->set_vehicle_state(ArticulatedRearChassisStatus529::SYSTEM_FAULT);

  int32_t cm = control_mode(bytes, length);
  if (cm == 0)
    status->set_control_mode(ArticulatedRearChassisStatus529::MODE_STANDBY);
  else if (cm == 1)
    status->set_control_mode(
        ArticulatedRearChassisStatus529::MODE_CAN_CONTROL);
  else
    status->set_control_mode(ArticulatedRearChassisStatus529::MODE_REMOTE);

  status->set_battery_voltage(battery_voltage(bytes, length));
  status->set_fault_high(fault_high(bytes, length));
  status->set_fault_low(fault_low(bytes, length));
  status->set_message_count(message_count(bytes, length));
}

int32_t RearChassisStatus529::vehicle_state(const std::uint8_t* bytes,
                                            int32_t length) const {
  // DBC: SG_ Vehicle_State : 7|8@0+ (1,0) [0|255]
  // Motorola big-endian: start bit 7 -> Byte 0
  return static_cast<int32_t>(bytes[0]);
}

int32_t RearChassisStatus529::control_mode(const std::uint8_t* bytes,
                                           int32_t length) const {
  // DBC: SG_ Control_Mode : 15|8@0+ (1,0) [0|255]
  // Motorola big-endian: start bit 15 -> Byte 1
  return static_cast<int32_t>(bytes[1]);
}

double RearChassisStatus529::battery_voltage(const std::uint8_t* bytes,
                                             int32_t length) const {
  // DBC: SG_ Battery_Voltage : 23|16@0+ (0.1,0) [0|65535] "V"
  // Motorola big-endian: Byte 2 = high, Byte 3 = low
  uint16_t raw = static_cast<uint16_t>((bytes[2] << 8) | bytes[3]);
  return raw * 0.1;
}

int32_t RearChassisStatus529::fault_high(const std::uint8_t* bytes,
                                         int32_t length) const {
  // DBC: SG_ Fault_High : 39|8@0+ (1,0) [0|255]
  // Byte 4
  return static_cast<int32_t>(bytes[4]);
}

int32_t RearChassisStatus529::fault_low(const std::uint8_t* bytes,
                                        int32_t length) const {
  // DBC: SG_ Fault_Low : 47|8@0+ (1,0) [0|255]
  // Byte 5
  return static_cast<int32_t>(bytes[5]);
}

int32_t RearChassisStatus529::message_count(const std::uint8_t* bytes,
                                            int32_t length) const {
  // DBC: SG_ Message_Count : 63|8@0+ (1,0) [0|255]
  // Byte 7
  return static_cast<int32_t>(bytes[7]);
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
