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
  )

set(plugin_HDR
  src/ForceShoePlugin.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(ForceShoePlugin "${plugin_SRC}" "${plugin_HDR}")

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
