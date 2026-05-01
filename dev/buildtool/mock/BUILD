
load("//tools/install:install.bzl", "install", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)


install(
    name = "mock_install",
    type = "neo",
    deps = ["//modules/common_msgs:install","//cyber:install","//modules/planning/cilqr_planner:install"],
)


install_src_files(
    name = "mock_install_src",
    type = "neo",
    deps = ["//modules/common_msgs:install_src","//cyber:install_src","//modules/planning/cilqr_planner:install_src"],
)
