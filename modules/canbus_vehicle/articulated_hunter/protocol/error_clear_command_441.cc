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

#include "modules/canbus_vehicle/articulated_hunter/protocol/error_clear_command_441.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Errorclearcommand441::ID = 0x441;

// public
Errorclearcommand441::Errorclearcommand441() { Reset(); }

uint32_t Errorclearcommand441::GetPeriod() const {
  // TODO(All) : modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Errorclearcommand441::Parse(const std::uint8_t* bytes, int32_t length,
                         ArticulatedHunter* chassis) const {
  chassis->mutable_error_clear_command_441()->set_clear_code(clear_code(bytes, length));
}

void Errorclearcommand441::UpdateData_Heartbeat(uint8_t* data) {
   // TODO(All) : you should add the heartbeat manually
}

void Errorclearcommand441::UpdateData(uint8_t* data) {
  set_p_clear_code(data, clear_code_);
}

void Errorclearcommand441::Reset() {
  // TODO(All) :  you should check this manually
  clear_code_ = Error_clear_command_441::CLEAR_CODE_CLEARTURNFAULT;
}

Errorclearcommand441* Errorclearcommand441::set_clear_code(
    Error_clear_command_441::Clear_codeType clear_code) {
  clear_code_ = clear_code;
  return this;
 }

// config detail: {'bit': 7, 'enum': {4: 'CLEAR_CODE_CLEARTURNFAULT', 5: 'CLEAR_CODE_CLEARRBFAULT', 6: 'CLEAR_CODE_CLEARLBFAULT', 255: 'CLEAR_CODE_CLEARALL'}, 'is_signed_var': False, 'len': 8, 'name': 'Clear_Code', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Errorclearcommand441::set_p_clear_code(uint8_t* data,
    Error_clear_command_441::Clear_codeType clear_code) {
  int x = clear_code;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Error_clear_command_441::Clear_codeType Errorclearcommand441::clear_code(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Error_clear_command_441::Clear_codeType ret =  static_cast<Error_clear_command_441::Clear_codeType>(x);
  return ret;
}
}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
