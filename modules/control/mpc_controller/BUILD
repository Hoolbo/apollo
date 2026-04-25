load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

MPC_COPTS = ['-DMODULE_NAME=\\"mpc_controller\\"']

cc_library(
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

cc_binary(
    name = "libmpc_controller_component.so",
    linkshared = True,
    linkstatic = True,
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

install(
    name = "_install_lib",
    targets = [":libmpc_controller_component.so"],
    data = ["cyberfile.xml"],
    library_dest = "lib/modules/control/mpc_controller",
    type = "neo",
    package_path = "modules/control/mpc_controller",
    visibility = ["//visibility:public"],
)

install(
    name = "_install_data",
    data = [":runtime_data"],
    data_dest = "share/modules/control/mpc_controller",
    type = "neo",
    package_path = "modules/control/mpc_controller",
    visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [":_install_lib", ":_install_data"],
    type = "neo",
    package_path = "modules/control/mpc_controller",
    visibility = ["//visibility:public"],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "src/modules/control/mpc_controller",
    filter = "*",
    type = "neo",
    visibility = ["//visibility:public"],
)
