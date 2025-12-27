load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_mycar",
    srcs = [
        "mycar_controller.cc",
        "mycar_message_manager.cc",
        "mycar_vehicle_factory.cc",
        "protocol/acu_drivemotor_563.cc",
        "protocol/acu_eps_547.cc",
        "protocol/drivemotor_acu_572.cc",
        "protocol/eps_acu_556.cc",
        "protocol/vcu_acu_general_524.cc",
    ],
    hdrs = [
        "mycar_controller.h",
        "mycar_message_manager.h",
        "mycar_vehicle_factory.h",
        "protocol/acu_drivemotor_563.h",
        "protocol/acu_eps_547.h",
        "protocol/drivemotor_acu_572.h",
        "protocol/eps_acu_556.h",
        "protocol/vcu_acu_general_524.h",
    ],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus_vehicle/mycar/proto:mycar_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_id_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libmycar_vehicle_factory.so",
    deps = [":apollo_canbus_vehicle_mycar"],
)

apollo_package()

cpplint()
