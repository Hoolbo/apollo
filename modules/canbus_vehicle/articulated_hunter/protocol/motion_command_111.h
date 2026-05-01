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

class Motioncommand111 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ArticulatedHunter> {
 public:
  static const int32_t ID;

  Motioncommand111();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ArticulatedHunter* chassis) const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'Target_Speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-4800|4800]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  Motioncommand111* set_target_speed(double target_speed);

  // config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'Target_Steer_Angle', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-400|400]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  Motioncommand111* set_target_steer_angle(double target_steer_angle);

 private:

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'Target_Speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-4800|4800]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  void set_p_target_speed(uint8_t* data, double target_speed);

  // config detail: {'bit': 55, 'is_signed_var': True, 'len': 16, 'name': 'Target_Steer_Angle', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-400|400]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  void set_p_target_steer_angle(uint8_t* data, double target_steer_angle);

  double target_speed(const std::uint8_t* bytes, const int32_t length) const;

  double target_steer_angle(const std::uint8_t* bytes, const int32_t length) const;

 private:
  double target_speed_;
  double target_steer_angle_;
};

}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo


