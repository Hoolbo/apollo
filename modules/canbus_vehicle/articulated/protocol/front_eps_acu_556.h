#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace articulated {

class FrontEpsAcu556
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::Articulated> {
 public:
  static const int32_t ID;

  void Parse(const std::uint8_t* bytes, int32_t length,
             Articulated* chassis) const override;

 private:
  double eps_angle(const std::uint8_t* bytes, int32_t length) const;
  bool eps_enable(const std::uint8_t* bytes, int32_t length) const;
  int32_t eps_error(const std::uint8_t* bytes, int32_t length) const;
  double eps_angle_speed(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
