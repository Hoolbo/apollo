#pragma once

#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

namespace apollo {
namespace canbus {
namespace articulated {

class ArticulatedMessageManager
    : public ::apollo::drivers::canbus::MessageManager<
          ::apollo::canbus::Articulated> {
 public:
  ArticulatedMessageManager();
  virtual ~ArticulatedMessageManager();
};

}  // namespace articulated
}  // namespace canbus
}  // namespace apollo
