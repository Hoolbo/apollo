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

#include "modules/canbus_vehicle/articulated_hunter/protocol/motor_feedback_high_2_252.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace articulated_hunter {

using ::apollo::drivers::canbus::Byte;

Motorfeedbackhigh2252::Motorfeedbackhigh2252() {}
const int32_t Motorfeedbackhigh2252::ID = 0x252;

void Motorfeedbackhigh2252::Parse(const std::uint8_t* bytes, int32_t length,
                         ArticulatedHunter* chassis) const {
  chassis->mutable_motor_feedback_high_2_252()->set_motor2_speed(motor2_speed(bytes, length));
  chassis->mutable_motor_feedback_high_2_252()->set_motor2_current(motor2_current(bytes, length));
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 16, 'name': 'motor2_speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32768|32767]', 'physical_unit': 'RPM', 'precision': 1.0, 'type': 'int'}
int Motorfeedbackhigh2252::motor2_speed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'motor2_current', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32768|32767]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Motorfeedbackhigh2252::motor2_current(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace articulated_hunter
}  // namespace canbus
}  // namespace apollo
