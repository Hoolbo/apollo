load("//tools/install:install.bzl", "install", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

INS_COPTS = ['-DMODULE_NAME=\\"ins\\"']

proto_library(
    name = "ins_conf_proto",
    srcs = ["proto/ins_conf.proto"],
)

cc_proto_library(
    name = "ins_conf_cc_proto",
    deps = [":ins_conf_proto"],
)

# Internal library for the component
cc_library(
    name = "ins_component_lib",
    srcs = ["ins_component.cc"],
    hdrs = ["ins_component.h"],
    copts = INS_COPTS,
    alwayslink = True,
    deps = [
        ":ins_conf_cc_proto",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/transform:apollo_transform",
        "@eigen",
    ],
)

# Shared library for the component
cc_binary(
    name = "libins_component.so",
    linkshared = True,
    linkstatic = True,
    tags = ["export_library", "ins"],
    deps = [":ins_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/*.yaml",
    ]) + ["cyberfile.xml"],
)

# Install targets
install(
    name = "_install_lib",
    targets = [":libins_component.so"],
    data = ["cyberfile.xml"],
    library_dest = "lib/modules/drivers/ins",
    type = "neo",
    package_path = "modules/drivers/ins",
    visibility = ["//visibility:public"],
)

install(
    name = "_install_data",
    data = [":runtime_data"],
    data_dest = "share/modules/drivers/ins",
    type = "neo",
    package_path = "modules/drivers/ins",
    visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [":_install_lib", ":_install_data"],
    type = "neo",
    package_path = "modules/drivers/ins",
    visibility = ["//visibility:public"],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "src/modules/drivers/ins",
    filter = "*",
    type = "neo",
    visibility = ["//visibility:public"],
)
