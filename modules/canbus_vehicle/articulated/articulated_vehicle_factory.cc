#include "modules/canbus_vehicle/articulated/articulated_vehicle_factory.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus_vehicle/articulated/articulated_controller.h"
#include "modules/canbus_vehicle/articulated/articulated_message_manager.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

namespace apollo {
namespace canbus {

using ::apollo::drivers::canbus::CanClientFactory;

bool ArticulatedVehicleFactory::Init(const CanbusConf* canbus_conf) {
  // 1. Init Can Client
  auto* can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ =
      can_factory->CreateCANClient(canbus_conf->can_card_parameter());
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
  AINFO << "Articulated message manager created.";

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
      node_->CreateWriter<::apollo::canbus::Articulated>(
          FLAGS_chassis_detail_topic);

  return true;
}

bool ArticulatedVehicleFactory::Start() {
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

void ArticulatedVehicleFactory::Stop() {
  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();
}

void ArticulatedVehicleFactory::UpdateCommand(
    const apollo::control::ControlCommand* control_command) {
  if (vehicle_controller_->Update(*control_command) != ErrorCode::OK) {
    AERROR << "Failed to process control command.";
  }
  can_sender_.Update();
}

void ArticulatedVehicleFactory::UpdateCommand(
    const apollo::external_command::ChassisCommand* chassis_command) {
  if (vehicle_controller_->Update(*chassis_command) != ErrorCode::OK) {
    AERROR << "Failed to process chassis command.";
  }
  can_sender_.Update();
}

Chassis ArticulatedVehicleFactory::publish_chassis() {
  return vehicle_controller_->chassis();
}

void ArticulatedVehicleFactory::PublishChassisDetail() {
  Articulated chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();
  chassis_detail_writer_->Write(chassis_detail);
}

void ArticulatedVehicleFactory::PublishChassisDetailSender() {}

void ArticulatedVehicleFactory::UpdateHeartbeat() {}

bool ArticulatedVehicleFactory::CheckChassisCommunicationFault() {
  return false;
}

void ArticulatedVehicleFactory::AddSendProtocol() {
  vehicle_controller_->AddSendMessage();
}

void ArticulatedVehicleFactory::ClearSendProtocol() {
  can_sender_.ClearMessage();
}

bool ArticulatedVehicleFactory::IsSendProtocolClear() {
  return can_sender_.IsMessageClear();
}

Chassis::DrivingMode ArticulatedVehicleFactory::Driving_Mode() {
  return vehicle_controller_->driving_mode();
}

std::unique_ptr<VehicleController<::apollo::canbus::Articulated>>
ArticulatedVehicleFactory::CreateVehicleController() {
  return std::unique_ptr<VehicleController<::apollo::canbus::Articulated>>(
      new articulated::ArticulatedController());
}

std::unique_ptr<MessageManager<::apollo::canbus::Articulated>>
ArticulatedVehicleFactory::CreateMessageManager() {
  return std::unique_ptr<MessageManager<::apollo::canbus::Articulated>>(
      new articulated::ArticulatedMessageManager());
}

}  // namespace canbus
}  // namespace apollo
