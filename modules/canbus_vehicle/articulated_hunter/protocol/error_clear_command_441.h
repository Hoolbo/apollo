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

class Errorclearcommand441 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;

  Errorclearcommand441();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ArticulatedHunter* chassis) const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'enum': {4: 'CLEAR_CODE_CLEARTURNFAULT', 5: 'CLEAR_CODE_CLEARRBFAULT', 6: 'CLEAR_CODE_CLEARLBFAULT', 255: 'CLEAR_CODE_CLEARALL'}, 'is_signed_var': False, 'len': 8, 'name': 'Clear_Code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  Errorclearcommand441* set_clear_code(Error_clear_command_441::Clear_codeType clear_code);

 private:

  // config detail: {'bit': 7, 'enum': {4: 'CLEAR_CODE_CLEARTURNFAULT', 5: 'CLEAR_CODE_CLEARRBFAULT', 6: 'CLEAR_CODE_CLEARLBFAULT', 255: 'CLEAR_CODE_CLEARALL'}, 'is_signed_var': False, 'len': 8, 'name': 'Clear_Code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_clear_code(uint8_t* data, Error_clear_command_441::Clear_codeType clear_code);

  Error_clear_command_441::Clear_codeType clear_code(const std::uint8_t* bytes, const int32_t length) const;

 private:
  Error_clear_command_441::Clear_codeType clear_code_;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo


