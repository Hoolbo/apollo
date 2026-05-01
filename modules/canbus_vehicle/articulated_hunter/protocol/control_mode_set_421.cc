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

#include "modules/canbus_vehicle/articulated_hunter/protocol/control_mode_set_421.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Controlmodeset421::ID = 0x421;

// public
Controlmodeset421::Controlmodeset421() { Reset(); }

uint32_t Controlmodeset421::GetPeriod() const {
  // TODO(All) : modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Controlmodeset421::Parse(const std::uint8_t* bytes, int32_t length,
                         ArticulatedHunter* chassis) const {
  chassis->mutable_control_mode_set_421()->set_mode_set(mode_set(bytes, length));
}

void Controlmodeset421::UpdateData_Heartbeat(uint8_t* data) {
   // TODO(All) : you should add the heartbeat manually
}

void Controlmodeset421::UpdateData(uint8_t* data) {
  set_p_mode_set(data, mode_set_);
}

void Controlmodeset421::Reset() {
  // TODO(All) :  you should check this manually
  mode_set_ = Control_mode_set_421::MODE_SET_STANDBY;
}

Controlmodeset421* Controlmodeset421::set_mode_set(
    Control_mode_set_421::Mode_setType mode_set) {
  mode_set_ = mode_set;
  return this;
 }

// config detail: {'bit': 7, 'enum': {0: 'MODE_SET_STANDBY', 1: 'MODE_SET_CANCONTROL', 2: 'MODE_SET_TOSTANDBY'}, 'is_signed_var': False, 'len': 8, 'name': 'Mode_Set', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Controlmodeset421::set_p_mode_set(uint8_t* data,
    Control_mode_set_421::Mode_setType mode_set) {
  int x = mode_set;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Control_mode_set_421::Mode_setType Controlmodeset421::mode_set(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Control_mode_set_421::Mode_setType ret =  static_cast<Control_mode_set_421::Mode_setType>(x);
  return ret;
}
}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
