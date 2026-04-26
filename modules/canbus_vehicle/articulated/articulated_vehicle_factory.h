#pragma once

#include <memory>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus_vehicle/articulated/proto/articulated.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"

#include "cyber/cyber.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/status/status.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

namespace apollo {
namespace canbus {

class ArticulatedVehicleFactory : public AbstractVehicleFactory {
 public:
  virtual ~ArticulatedVehicleFactory() = default;

  bool Init(const CanbusConf* canbus_conf) override;
  bool Start() override;
  void Stop() override;

  void UpdateCommand(
      const apollo::control::ControlCommand* control_command) override;

  void UpdateCommand(
      const apollo::external_command::ChassisCommand* chassis_command) override;

  Chassis publish_chassis() override;
  void PublishChassisDetail() override;
  void PublishChassisDetailSender() override;
  void UpdateHeartbeat() override;
  bool CheckChassisCommunicationFault() override;
  void AddSendProtocol() override;
  void ClearSendProtocol() override;
  bool IsSendProtocolClear() override;
  Chassis::DrivingMode Driving_Mode() override;

 private:
  std::unique_ptr<VehicleController<::apollo::canbus::Articulated>>
  CreateVehicleController();

  std::unique_ptr<MessageManager<::apollo::canbus::Articulated>>
  CreateMessageManager();

  std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
  std::unique_ptr<apollo::drivers::canbus::CanClient> can_client_;
  CanSender<::apollo::canbus::Articulated> can_sender_;
  apollo::drivers::canbus::CanReceiver<::apollo::canbus::Articulated>
      can_receiver_;
  std::unique_ptr<MessageManager<::apollo::canbus::Articulated>>
      message_manager_;
  std::unique_ptr<VehicleController<::apollo::canbus::Articulated>>
      vehicle_controller_;

  std::shared_ptr<
      ::apollo::cyber::Writer<::apollo::canbus::Articulated>>
      chassis_detail_writer_;
};

CYBER_REGISTER_VEHICLEFACTORY(ArticulatedVehicleFactory)

}  // namespace canbus
}  // namespace apollo
