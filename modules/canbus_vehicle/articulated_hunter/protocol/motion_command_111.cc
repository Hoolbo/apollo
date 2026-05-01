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

#include "modules/canbus_vehicle/articulated_hunter/protocol/motion_command_111.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

const int32_t Motioncommand111::ID = 0x111;

// public
Motioncommand111::Motioncommand111() { Reset(); }

uint32_t Motioncommand111::GetPeriod() const {
  // TODO(All) : modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Motioncommand111::Parse(const std::uint8_t* bytes, int32_t length,
                         ArticulatedHunter* chassis) const {
  chassis->mutable_motion_command_111()->set_target_speed(target_speed(bytes, length));
  chassis->mutable_motion_command_111()->set_target_steer_angle(target_steer_angle(bytes, length));
}

void Motioncommand111::UpdateData_Heartbeat(uint8_t* data) {
   // TODO(All) : you should add the heartbeat manually
}

void Motioncommand111::UpdateData(uint8_t* data) {
  set_p_target_speed(data, target_speed_);
  set_p_target_steer_angle(data, target_steer_angle_);
}

void Motioncommand111::Reset() {
  // TODO(All) :  you should check this manually
  target_speed_ = 0.0;
  target_steer_angle_ = 0.0;
}

Motioncommand111* Motioncommand111::set_target_speed(
    double target_speed) {
  target_speed_ = target_speed;
  return this;
 }

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'Target_Speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-4800|4800]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
void Motioncommand111::set_p_target_speed(uint8_t* data,
    double target_speed) {
  target_speed = ProtocolData::BoundedValue(-4800.0, 4800.0, target_speed);
  int x = target_speed / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 0);
  to_set1.set_value(t, 0, 8);
}


Motioncommand111* Motioncommand111::set_target_steer_angle(
    double target_steer_angle) {
  target_steer_angle_ = target_steer_angle;
  return this;
 }

// config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'Target_Steer_Angle', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-400|400]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
void Motioncommand111::set_p_target_steer_angle(uint8_t* data,
    double target_steer_angle) {
  target_steer_angle = ProtocolData::BoundedValue(-400.0, 400.0, target_steer_angle);
  int x = target_steer_angle / 0.001000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 7);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 6);
  to_set1.set_value(t, 0, 8);
}


double Motioncommand111::target_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}

double Motioncommand111::target_steer_angle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 7);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
