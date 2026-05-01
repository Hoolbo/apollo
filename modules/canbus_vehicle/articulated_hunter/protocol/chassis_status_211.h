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

#pragma once

#include "modules/canbus_vehicle/articulated_hunter/proto/articulated_hunter.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

class Chassisstatus211 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;
  Chassisstatus211();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ArticulatedHunter* chassis) const override;

 private:

    // config detail: {'bit': 7, 'enum': {0: 'VEHICLE_STATE_NORMAL', 1: 'VEHICLE_STATE_EMERGENCYSTOP', 2: 'VEHICLE_STATE_SYSTEMFAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Vehicle_State', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
    Chassis_status_211::Vehicle_stateType vehicle_state(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 15, 'enum': {0: 'CONTROL_MODE_STANDBY', 1: 'CONTROL_MODE_CANCONTROL', 2: 'CONTROL_MODE_REMOTE'}, 'is_signed_var': False, 'len': 8, 'name': 'Control_Mode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
    Chassis_status_211::Control_modeType control_mode(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 23, 'is_signed_var': False, 'len': 16, 'name': 'Battery_Voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
    double battery_voltage(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 39, 'is_signed_var': False, 'len': 8, 'name': 'Fault_High', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
    int fault_high(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'Fault_Low', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
    int fault_low(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'Reserved_6', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
    int reserved_6(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'Message_Count', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
    int message_count(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo


