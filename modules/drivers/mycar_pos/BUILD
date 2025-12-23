load("//tools/install:install.bzl", "install", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

MYCAR_POS_COPTS = ['-DMODULE_NAME=\\"mycar_pos\\"']

proto_library(
    name = "mycar_pos_conf_proto",
    srcs = ["proto/mycar_pos_conf.proto"],
)

cc_proto_library(
    name = "mycar_pos_conf_cc_proto",
    deps = [":mycar_pos_conf_proto"],
)

# Internal library for the component
cc_library(
    name = "mycar_pos_component_lib",
    srcs = ["mycar_pos_component.cc"],
    hdrs = ["mycar_pos_component.h"],
    copts = MYCAR_POS_COPTS,
    alwayslink = True,
    deps = [
        ":mycar_pos_conf_cc_proto",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/transform:apollo_transform",
        "@eigen",
    ],
)

# Shared library for the component
cc_binary(
    name = "libmycar_pos_component.so",
    linkshared = True,
    linkstatic = True,
    tags = ["export_library", "mycar_pos"],
    deps = [":mycar_pos_component_lib"],
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
    targets = [":libmycar_pos_component.so"],
    data = ["cyberfile.xml"],
    library_dest = "lib/modules/drivers/mycar_pos",
    type = "neo",
    package_path = "modules/drivers/mycar_pos",
    visibility = ["//visibility:public"],
)

install(
    name = "_install_data",
    data = [":runtime_data"],
    data_dest = "share/modules/drivers/mycar_pos",
    type = "neo",
    package_path = "modules/drivers/mycar_pos",
    visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [":_install_lib", ":_install_data"],
    type = "neo",
    package_path = "modules/drivers/mycar_pos",
    visibility = ["//visibility:public"],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "src/modules/drivers/mycar_pos",
    filter = "*",
    type = "neo",
    visibility = ["//visibility:public"],
)
