cmake_minimum_required(VERSION 3.1)

project(ForceShoePlugin LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/ForceShoePlugin.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceShoePlugin.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ForceShoePlugin.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC
  src/ForceShoePlugin.cpp
  src/cmt1.cpp
  src/cmt2.cpp
  src/cmt3.cpp
  src/cmtmessage.cpp
  src/cmtpacket.cpp
  src/cmtscan.cpp
  src/xsens_exception.cpp
  src/xsens_std.cpp
  src/xsens_time.cpp
  )

set(plugin_HDR
  src/ForceShoePlugin.h
  src/cmt1.h
  src/cmt2.h
  src/cmt3.h
  src/cmtdef.h
  src/cmtmessage.h
  src/cmtpacket.h
  src/cmtscan.h
  src/pstdint.h
  src/xsens_exception.h
  src/xsens_fifoqueue.h
  src/xsens_file.h
  src/xsens_janitors.h
  src/xsens_list.h
  src/xsens_std.h
  src/xsens_time.h
)

# The add_plugin macro is provided by mc_rtc
option(AUTOLOAD_ForceShoePlugin_PLUGIN "Automatically load ${PLUGIN_NAME} plugin" OFF)
add_plugin(ForceShoePlugin "${plugin_SRC}" "${plugin_HDR}")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
