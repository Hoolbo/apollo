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

#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_low_2_262.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

Motorfeedbacklow2262::Motorfeedbacklow2262() {}
const int32_t Motorfeedbacklow2262::ID = 0x262;

void Motorfeedbacklow2262::Parse(const std::uint8_t* bytes, int32_t length,
                         ArticulatedHunter* chassis) const {
  chassis->mutable_motor_feedback_low_2_262()->set_driver_voltage_2(driver_voltage_2(bytes, length));
  chassis->mutable_motor_feedback_low_2_262()->set_driver_temp_2(driver_temp_2(bytes, length));
  chassis->mutable_motor_feedback_low_2_262()->set_motor_temp_2(motor_temp_2(bytes, length));
  chassis->mutable_motor_feedback_low_2_262()->set_driver_status_2(driver_status_2(bytes, length));
}

// config detail: {'bit': 7, 'is_signed_var': False, 'len': 16, 'name': 'driver_voltage_2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|65535]', 'physical_unit': 'V', 'precision': 0.1, 'type': 'double'}
double Motorfeedbacklow2262::driver_voltage_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'driver_temp_2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32768|32767]', 'physical_unit': 'degC', 'precision': 1.0, 'type': 'int'}
int Motorfeedbacklow2262::driver_temp_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'is_signed_var': True, 'len': 8, 'name': 'motor_temp_2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-128|127]', 'physical_unit': 'degC', 'precision': 1.0, 'type': 'int'}
int Motorfeedbacklow2262::motor_temp_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  x <<= 24;
  x >>= 24;

  int ret = x;
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'driver_status_2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Motorfeedbacklow2262::driver_status_2(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
