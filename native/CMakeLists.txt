cmake_minimum_required(VERSION 3.20)
project(umi_vive_linux C)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LIBSURVIVE REQUIRED IMPORTED_TARGET survive)

add_executable(umi-vive-linux vive_linux_sender.c)
target_compile_features(umi-vive-linux PRIVATE c_std_11)
target_compile_options(umi-vive-linux PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(umi-vive-linux PRIVATE PkgConfig::LIBSURVIVE Threads::Threads m)
target_compile_definitions(
  umi-vive-linux
  PRIVATE UMI_DEFAULT_SURVIVE_PLUGIN_DIR="${UMI_LIBSURVIVE_PREFIX}/lib/libsurvive/plugins"
)
set_target_properties(
  umi-vive-linux
  PROPERTIES INSTALL_RPATH "\$ORIGIN/../../.deps/libsurvive/lib"
)

install(TARGETS umi-vive-linux RUNTIME DESTINATION bin)
