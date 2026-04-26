#include "modules/canbus_vehicle/articulated/protocol/front_acu_eps_547.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace articulated {

using ::apollo::drivers::canbus::Byte;

const int32_t FrontAcuEps547::ID = 0x223;

FrontAcuEps547::FrontAcuEps547() { Reset(); }

void FrontAcuEps547::Parse(const std::uint8_t* bytes, int32_t length,
                           Articulated* chassis) const {}

void FrontAcuEps547::UpdateData(uint8_t* data) {
  set_acu_eps_angle(data, eps_angle_);
  set_acu_eps_enable(data, eps_enable_);
  set_acu_eps_angle_speed(data, eps_angle_speed_);
}

void FrontAcuEps547::Reset() {
  eps_angle_ = 0.0;
  eps_enable_ = 0;
  eps_angle_speed_ = 0.0;
}

void FrontAcuEps547::set_acu_eps_angle(uint8_t* data, double angle) {
  // DBC: 40|16@1+ (1,-1024) [-324|1724]
  angle = ProtocolData::BoundedValue(-324.0, 1724.0, angle);
  int32_t x = static_cast<int32_t>(angle + 1024.0);

  Byte frame(data + 5);
  frame.set_value(x, 0, 8);
  Byte frame_high(data + 6);
  frame_high.set_value(x >> 8, 0, 8);
}

void FrontAcuEps547::set_acu_eps_enable(uint8_t* data, int enable) {
  // DBC: 56|1@1+ (Byte 7 bit 0)
  Byte frame(data + 7);
  frame.set_value(enable, 0, 1);
}

void FrontAcuEps547::set_acu_eps_angle_speed(uint8_t* data, double speed) {
  // DBC: 32|8@1+ (2.0,0) [0|180]
  speed = ProtocolData::BoundedValue(0.0, 180.0, speed);
  int32_t x = static_cast<int32_t>(speed / 2.0);
  Byte frame(data + 4);
  frame.set_value(x, 0, 8);
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
