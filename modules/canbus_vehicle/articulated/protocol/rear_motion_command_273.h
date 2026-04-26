#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

// Hunter SE Motion Command (0x111 / 273)
// Motorola byte order, signed 16-bit, factor=0.001
class RearMotionCommand273
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  RearMotionCommand273();

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  void set_target_speed(double speed) { target_speed_ = speed; }
  void set_target_steer_angle(double angle) { target_steer_angle_ = angle; }

 private:
  double target_speed_ = 0.0;       // m/s
  double target_steer_angle_ = 0.0; // rad
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
