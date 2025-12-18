#pragma once

#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/canbus_vehicle/mycar/proto/mycar.pb.h"

namespace apollo {
namespace canbus {
namespace mycar {

class MycarMessageManager
    : public ::apollo::drivers::canbus::MessageManager<
          ::apollo::canbus::Mycar> {
 public:
  MycarMessageManager();
  virtual ~MycarMessageManager();
};

}  // namespace mycar
}  // namespace canbus
}  // namespace apollo
