load("//third_party/rules_proto:init.bzl", apollo_neo_3rd_rules_proto_repo = "init")
load("//third_party/eigen3:init.bzl", apollo_neo_3rd_eigen3_repo = "init")
load("//third_party/grpc:init.bzl", apollo_neo_3rd_grpc_repo = "init")
load("//third_party/bazel_skylib:init.bzl", apollo_neo_3rd_bazel_skylib_repo = "init")
load("//third_party/nlohmann_json:init.bzl", apollo_neo_3rd_nlohmann_json_repo = "init")
load("//third_party/protobuf:init.bzl", apollo_neo_3rd_protobuf_repo = "init")
load("//third_party/gtest:init.bzl", apollo_neo_3rd_gtest_repo = "init")
load("//third_party/rules_python:init.bzl", apollo_neo_3rd_rules_python_repo = "init")
load("//third_party/cpplint:init.bzl", apollo_neo_3rd_cpplint_repo = "init")
load("//third_party/gpus:init.bzl", apollo_neo_3rd_gpus_repo = "init")
load("//third_party/py:init.bzl", apollo_neo_3rd_py_repo = "init")
def clean_dep(dep):
    return str(Label(dep))
def system_libncurses5_dev_repo():
    native.new_local_repository(
        name = "ncurses5",
        build_file = clean_dep("//dev/bazel:libncurses5-dev.BUILD"),
        path = "/usr/include",
    )
def system_libgtk20_0_repo():
    native.new_local_repository(
        name = "libgtk2.0-0",
        build_file = clean_dep("//dev/bazel:libgtk2.0-0.BUILD"),
        path = "/usr/include",
    )
def system_bvar_repo():
    native.new_local_repository(
        name = "bvar",
        build_file = clean_dep("//dev/bazel:bvar.BUILD"),
        path = "/usr/include",
    )
def apollo_neo_3rd_fastdds_wrap_repo():
    native.new_local_repository(
        name = "fastdds",
        build_file = clean_dep("//dev/bazel:3rd-fastdds-wrap.BUILD"),
        path = "/opt/apollo/neo/packages/3rd-fastdds-wrap/latest",
    )
def system_sysstat_repo():
    native.new_local_repository(
        name = "sysstat",
        build_file = clean_dep("//dev/bazel:sysstat.BUILD"),
        path = "/usr/include",
    )
def system_gperftools_repo():
    native.new_local_repository(
        name = "gperftools",
        build_file = clean_dep("//dev/bazel:gperftools.BUILD"),
        path = "/usr/include",
    )
def system_libunwind_dev_repo():
    native.new_local_repository(
        name = "libunwind-dev",
        build_file = clean_dep("//dev/bazel:libunwind-dev.BUILD"),
        path = "/usr/include",
    )
def system_uuid_dev_repo():
    native.new_local_repository(
        name = "uuid",
        build_file = clean_dep("//dev/bazel:uuid-dev.BUILD"),
        path = "/usr/include",
    )
def system_nethogs_repo():
    native.new_local_repository(
        name = "nethogs",
        build_file = clean_dep("//dev/bazel:nethogs.BUILD"),
        path = "/usr/include",
    )
def apollo_neo_3rd_proj_repo():
    native.new_local_repository(
        name = "proj",
        build_file = clean_dep("//dev/bazel:3rd-proj.BUILD"),
        path = "/opt/apollo/neo/packages/3rd-proj/latest",
    )
def apollo_neo_3rd_gflags_repo():
    native.new_local_repository(
        name = "com_github_gflags_gflags",
        build_file = clean_dep("//dev/bazel:3rd-gflags.BUILD"),
        path = "/opt/apollo/neo/packages/3rd-gflags/latest",
    )
def apollo_neo_3rd_glog_repo():
    native.new_local_repository(
        name = "com_github_google_glog",
        build_file = clean_dep("//dev/bazel:3rd-glog.BUILD"),
        path = "/opt/apollo/neo/packages/3rd-glog/latest",
    )
def system_libwebp_dev_repo():
    native.new_local_repository(
        name = "libwebp-dev",
        build_file = clean_dep("//dev/bazel:libwebp-dev.BUILD"),
        path = "/usr/include",
    )
def apollo_neo_3rd_opencv_repo():
    native.new_local_repository(
        name = "opencv",
        build_file = clean_dep("//dev/bazel:3rd-opencv.BUILD"),
        path = "/opt/apollo/neo/packages/3rd-opencv/latest",
    )
def system_libtinyxml2_dev_repo():
    native.new_local_repository(
        name = "tinyxml2",
        build_file = clean_dep("//dev/bazel:libtinyxml2-dev.BUILD"),
        path = "/usr/include",
    )
def init_deps():
    apollo_neo_3rd_rules_proto_repo()
    apollo_neo_3rd_eigen3_repo()
    apollo_neo_3rd_grpc_repo()
    apollo_neo_3rd_bazel_skylib_repo()
    apollo_neo_3rd_nlohmann_json_repo()
    apollo_neo_3rd_protobuf_repo()
    apollo_neo_3rd_gtest_repo()
    apollo_neo_3rd_rules_python_repo()
    apollo_neo_3rd_cpplint_repo()
    apollo_neo_3rd_gpus_repo()
    apollo_neo_3rd_py_repo()
    system_libncurses5_dev_repo()
    system_libgtk20_0_repo()
    system_bvar_repo()
    apollo_neo_3rd_fastdds_wrap_repo()
    system_sysstat_repo()
    system_gperftools_repo()
    system_libunwind_dev_repo()
    system_uuid_dev_repo()
    system_nethogs_repo()
    apollo_neo_3rd_proj_repo()
    apollo_neo_3rd_gflags_repo()
    apollo_neo_3rd_glog_repo()
    system_libwebp_dev_repo()
    apollo_neo_3rd_opencv_repo()
    system_libtinyxml2_dev_repo()