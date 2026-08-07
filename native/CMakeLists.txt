cmake_minimum_required(VERSION 3.20)
project(umi_vive_linux C)

find_package(Threads REQUIRED)

if(NOT UMI_LIBSURVIVE_PREFIX)
  message(FATAL_ERROR "UMI_LIBSURVIVE_PREFIX must point to the isolated libsurvive install")
endif()
find_path(
  LIBSURVIVE_INCLUDE_DIR
  NAMES survive.h
  PATHS "${UMI_LIBSURVIVE_PREFIX}/include/libsurvive"
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(
  LIBSURVIVE_LIBRARY
  NAMES survive
  PATHS "${UMI_LIBSURVIVE_PREFIX}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

add_executable(umi-vive-linux vive_linux_sender.c)
target_compile_features(umi-vive-linux PRIVATE c_std_11)
target_compile_options(umi-vive-linux PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(
  umi-vive-linux
  SYSTEM
  PRIVATE
    "${UMI_LIBSURVIVE_PREFIX}/include"
    "${LIBSURVIVE_INCLUDE_DIR}"
    "${LIBSURVIVE_INCLUDE_DIR}/redist"
)
target_link_libraries(umi-vive-linux PRIVATE "${LIBSURVIVE_LIBRARY}" Threads::Threads m)
target_compile_definitions(
  umi-vive-linux
  PRIVATE UMI_DEFAULT_SURVIVE_PLUGIN_DIR="${UMI_LIBSURVIVE_PREFIX}/lib/libsurvive/plugins"
)
set_target_properties(
  umi-vive-linux
  PROPERTIES INSTALL_RPATH "\$ORIGIN/../../.deps/libsurvive/lib"
)

install(TARGETS umi-vive-linux RUNTIME DESTINATION bin)
