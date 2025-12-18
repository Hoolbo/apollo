#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus_vehicle/mycar/proto/mycar.pb.h"

namespace apollo {
namespace canbus {
namespace mycar {

class EpsAcu556 : public ::apollo::drivers::canbus::ProtocolData<
                      ::apollo::canbus::Mycar> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Mycar* chassis) const override;

 private:
  double eps_angle(const std::uint8_t* bytes, int32_t length) const;
  bool eps_enable(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
