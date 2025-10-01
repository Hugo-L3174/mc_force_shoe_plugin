cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)

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
set(xsens_sdk_SRC
  src/xsens_sdk/cmt1.cpp
  src/xsens_sdk/cmt1.h
  src/xsens_sdk/cmt2.cpp
  src/xsens_sdk/cmt2.h
  src/xsens_sdk/cmt3.cpp
  src/xsens_sdk/cmt3.h
  src/xsens_sdk/cmtdef.h
  src/xsens_sdk/cmtmessage.cpp
  src/xsens_sdk/cmtmessage.h
  src/xsens_sdk/cmtpacket.cpp
  src/xsens_sdk/cmtpacket.h
  src/xsens_sdk/cmtscan.cpp
  src/xsens_sdk/cmtscan.h
  src/xsens_sdk/pstdint.h
  src/xsens_sdk/xsens_exception.cpp
  src/xsens_sdk/xsens_exception.h
  src/xsens_sdk/xsens_fifoqueue.h
  src/xsens_sdk/xsens_file.h
  src/xsens_sdk/xsens_janitors.h
  src/xsens_sdk/xsens_list.h
  src/xsens_sdk/xsens_std.cpp
  src/xsens_sdk/xsens_std.h
  src/xsens_sdk/xsens_time.cpp
  src/xsens_sdk/xsens_time.h
)
add_library(xsens_sdk OBJECT ${xsens_sdk_SRC})
set_target_properties(xsens_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(xsens_sdk INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/xsens_sdk)

set(plugin_SRC
  src/ForceShoePlugin.cpp
)

set(plugin_HDR
  src/ForceShoePlugin.h
)

# The add_plugin macro is provided by mc_rtc
set(AUTOLOAD_ForceShoePlugin_PLUGIN OFF
    CACHE INTERNAL "Automatically load ForceShoePlugin plugin")
add_plugin(ForceShoePlugin "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(ForceShoePlugin PUBLIC xsens_sdk)
target_include_directories(ForceShoePlugin PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
