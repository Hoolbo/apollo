#pragma once

#include "modules/canbus_vehicle/mycar/proto/mycar.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace mycar {

class AcuEps547
    : public ::apollo::drivers::canbus::ProtocolData<::apollo::canbus::Mycar> {
 public:
  static const int32_t ID;

  AcuEps547();

  void Parse(const std::uint8_t* bytes, int32_t length,
             Mycar* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  void set_eps_angle(double angle) { eps_angle_ = angle; }
  void set_eps_enable(bool enable) { eps_enable_ = enable ? 1 : 0; }

 private:
  void set_acu_eps_angle(uint8_t* data, double angle);
  void set_acu_eps_enable(uint8_t* data, int enable);
  void set_acu_eps_angle_speed(uint8_t* data, double speed);

  double eps_angle_ = 0.0;
  int eps_enable_ = 0;
  double eps_angle_speed_ = 0.0;
};

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
