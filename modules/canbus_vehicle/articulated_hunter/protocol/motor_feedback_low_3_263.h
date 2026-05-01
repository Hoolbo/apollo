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

class Motorfeedbacklow3263 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;
  Motorfeedbacklow3263();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ArticulatedHunter* chassis) const override;

 private:

    // config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'Driver_Voltage_3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
    double driver_voltage_3(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'Driver_Temp_3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32768|32767]', 'physical_unit': 'degC', 'precision': 1.0, 'type': 'int'}
    int driver_temp_3(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 39, 'is_signed_var': True, 'len': 8, 'name': 'Motor_Temp_3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-128|127]', 'physical_unit': 'degC', 'precision': 1.0, 'type': 'int'}
    int motor_temp_3(const std::uint8_t* bytes, const int32_t length) const;

    // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'Driver_Status_3', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
    int driver_status_3(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo


