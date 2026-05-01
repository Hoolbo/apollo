load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MPC_COPTS = ['-DMODULE_NAME=\\"mpc_controller\\"']

apollo_cc_library(
    name = "mpc_controller_component_lib",
    srcs = ["mpc_controller_component.cc"],
    hdrs = ["mpc_controller_component.h"],
    copts = MPC_COPTS + [
        "-std=c++17",
        "-Wno-error=unused-but-set-variable",
    ],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_github_nlohmann_json//:json",
        "@eigen",
        "@osqp//:osqp",
    ],
)

apollo_component(
    name = "libmpc_controller_component.so",
    deps = [":mpc_controller_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]) + ["cyberfile.xml"],
)

apollo_package()

cpplint()
