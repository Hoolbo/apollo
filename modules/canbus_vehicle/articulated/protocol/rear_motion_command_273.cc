#include "modules/canbus_vehicle/articulated/protocol/rear_motion_command_273.h"

#include "glog/logging.h"

namespace apollo {
namespace canbus {
namespace articulated {

const int32_t RearMotionCommand273::ID = 0x111;

RearMotionCommand273::RearMotionCommand273() { Reset(); }

void RearMotionCommand273::Parse(const std::uint8_t* bytes, int32_t length,
                                 Articulated* chassis) const {
  // This is a send-only message, no parsing needed
}

void RearMotionCommand273::UpdateData(uint8_t* data) {
  // DBC: SG_ Target_Speed : 7|16@0- (0.001,0) [-4800|4800] "m/s"
  // Motorola big-endian: Byte 0 = high, Byte 1 = low
  // Signed 16-bit, factor = 0.001
  double speed = target_speed_;
  if (speed > 4.8) speed = 4.8;
  if (speed < -4.8) speed = -4.8;
  int16_t raw_speed = static_cast<int16_t>(speed / 0.001);
  data[0] = static_cast<uint8_t>((raw_speed >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(raw_speed & 0xFF);

  // DBC: SG_ Target_Steer_Angle : 55|16@0- (0.001,0) [-400|400] "rad"
  // Motorola big-endian: start bit 55 -> Byte 6 = high, Byte 7 = low
  double steer = target_steer_angle_;
  if (steer > 0.4) steer = 0.4;
  if (steer < -0.4) steer = -0.4;
  int16_t raw_steer = static_cast<int16_t>(steer / 0.001);
  data[6] = static_cast<uint8_t>((raw_steer >> 8) & 0xFF);
  data[7] = static_cast<uint8_t>(raw_steer & 0xFF);

  AINFO << "RearMotionCommand273: speed=" << target_speed_
        << " m/s, steer=" << target_steer_angle_ << " rad";
}

void RearMotionCommand273::Reset() {
  target_speed_ = 0.0;
  target_steer_angle_ = 0.0;
}

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
