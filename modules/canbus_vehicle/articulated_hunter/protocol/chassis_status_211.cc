/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus_vehicle/articulated_hunter/protocol/chassis_status_211.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

Chassisstatus211::Chassisstatus211() {}
const int32_t Chassisstatus211::ID = 0x211;

void Chassisstatus211::Parse(const std::uint8_t* bytes, int32_t length,
                         ArticulatedHunter* chassis) const {
  chassis->mutable_chassis_status_211()->set_vehicle_state(vehicle_state(bytes, length));
  chassis->mutable_chassis_status_211()->set_control_mode(control_mode(bytes, length));
  chassis->mutable_chassis_status_211()->set_battery_voltage(battery_voltage(bytes, length));
  chassis->mutable_chassis_status_211()->set_fault_high(fault_high(bytes, length));
  chassis->mutable_chassis_status_211()->set_fault_low(fault_low(bytes, length));
  chassis->mutable_chassis_status_211()->set_reserved_6(reserved_6(bytes, length));
  chassis->mutable_chassis_status_211()->set_message_count(message_count(bytes, length));
}

// config detail: {'bit': 7, 'enum': {0: 'VEHICLE_STATE_NORMAL', 1: 'VEHICLE_STATE_EMERGENCYSTOP', 2: 'VEHICLE_STATE_SYSTEMFAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'vehicle_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_211::Vehicle_stateType Chassisstatus211::vehicle_state(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_211::Vehicle_stateType ret =  static_cast<Chassis_status_211::Vehicle_stateType>(x);
  return ret;
}

// config detail: {'bit': 15, 'enum': {0: 'CONTROL_MODE_STANDBY', 1: 'CONTROL_MODE_CANCONTROL', 2: 'CONTROL_MODE_REMOTE'}, 'is_signed_var': False, 'len': 8, 'name': 'control_mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
Chassis_status_211::Control_modeType Chassisstatus211::control_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Chassis_status_211::Control_modeType ret =  static_cast<Chassis_status_211::Control_modeType>(x);
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Chassisstatus211::battery_voltage(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name': 'fault_high', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassisstatus211::fault_high(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'fault_low', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassisstatus211::fault_low(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'reserved_6', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassisstatus211::reserved_6(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'message_count', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Chassisstatus211::message_count(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
