#include "modules/canbus_vehicle/articulated_hunter/articulated_hunter_vehicle_factory.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus_vehicle/articulated_hunter/articulated_hunter_controller.h"
#include "modules/canbus_vehicle/articulated_hunter/articulated_hunter_message_manager.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

namespace apollo {
namespace canbus {

using ::apollo::drivers::canbus::CanClientFactory;

bool ArticulatedHunterVehicleFactory::Init(const CanbusConf* canbus_conf) {
  // 1. Init Can Client
  auto* can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(canbus_conf->can_card_parameter());
  if (!can_client_) {
    AERROR << "Failed to create can client.";
    return false;
  }

  // 2. Init Message Manager
  message_manager_ = CreateMessageManager();
  if (message_manager_ == nullptr) {
    AERROR << "Failed to create message manager.";
    return false;
  }
  AINFO << "Message manager created.";

  // 3. Init Vehicle Controller
  vehicle_controller_ = CreateVehicleController();
  if (vehicle_controller_ == nullptr) {
    AERROR << "Failed to create vehicle controller.";
    return false;
  }

  // 4. Init sender/receiver
  if (can_sender_.Init(can_client_.get(), message_manager_.get(),
                       canbus_conf->enable_sender_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can sender.";
    return false;
  }

  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         canbus_conf->enable_receiver_log()) != ErrorCode::OK) {
    AERROR << "Failed to init can receiver.";
    return false;
  }

  // 5. Init Controller
  if (vehicle_controller_->Init(canbus_conf->vehicle_parameter(), &can_sender_,
                                message_manager_.get()) != ErrorCode::OK) {
    AERROR << "Failed to init vehicle controller.";
    return false;
  }

  // 6. Create node for chassis detail
  node_ = ::apollo::cyber::CreateNode("chassis_detail");
  chassis_detail_writer_ =
      node_->CreateWriter<::apollo::canbus::ArticulatedHunter>(FLAGS_chassis_detail_topic);

  return true;
}

bool ArticulatedHunterVehicleFactory::Start() {
  if (can_client_->Start() != ErrorCode::OK) {
    AERROR << "Failed to start can client.";
    return false;
  }
  if (can_receiver_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can receiver.";
    return false;
  }
  if (can_sender_.Start() != ErrorCode::OK) {
    AERROR << "Failed to start can sender.";
    return false;
  }
  if (!vehicle_controller_->Start()) {
    AERROR << "Failed to start vehicle controller.";
    return false;
  }
  return true;
}

void ArticulatedHunterVehicleFactory::Stop() {
  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();
}

void ArticulatedHunterVehicleFactory::UpdateCommand(
    const apollo::control::ControlCommand* control_command) {
  if (vehicle_controller_->Update(*control_command) != ErrorCode::OK) {
    AERROR << "Failed to process control command.";
  }
  can_sender_.Update();
}

void ArticulatedHunterVehicleFactory::UpdateCommand(
    const apollo::external_command::ChassisCommand* chassis_command) {
  if (vehicle_controller_->Update(*chassis_command) != ErrorCode::OK) {
    AERROR << "Failed to process chassis command.";
  }
  can_sender_.Update();
}

Chassis ArticulatedHunterVehicleFactory::publish_chassis() {
  return vehicle_controller_->chassis();
}

void ArticulatedHunterVehicleFactory::PublishChassisDetail() {
  ArticulatedHunter chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();
  chassis_detail_writer_->Write(chassis_detail);
}

void ArticulatedHunterVehicleFactory::PublishChassisDetailSender() {}

void ArticulatedHunterVehicleFactory::UpdateHeartbeat() {}

bool ArticulatedHunterVehicleFactory::CheckChassisCommunicationFault() { return false; }

void ArticulatedHunterVehicleFactory::AddSendProtocol() {
  vehicle_controller_->AddSendMessage();
}

void ArticulatedHunterVehicleFactory::ClearSendProtocol() { can_sender_.ClearMessage(); }

bool ArticulatedHunterVehicleFactory::IsSendProtocolClear() {
  return can_sender_.IsMessageClear();
}

Chassis::DrivingMode ArticulatedHunterVehicleFactory::Driving_Mode() {
  return vehicle_controller_->driving_mode();
}

std::unique_ptr<VehicleController<::apollo::canbus::ArticulatedHunter>>
ArticulatedHunterVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController<::apollo::canbus::ArticulatedHunter>>(
      new articulated_hunter::ArticulatedHunterController());
}

std::unique_ptr<MessageManager<::apollo::canbus::ArticulatedHunter>>
ArticulatedHunterVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::apollo::canbus::ArticulatedHunter>>(
      new articulated_hunter::ArticulatedHunterMessageManager());
}

}  // namespace canbus
}  // namespace apollo
