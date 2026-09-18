cmake_minimum_required(VERSION 3.8)
project(diffbot_gpio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# ── Standalone GPIO node executable ──────────────────────────────────────────
add_executable(diffbot_gpio_node
  src/diffbot_gpio.cpp
)

target_include_directories(diffbot_gpio_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(diffbot_gpio_node
  pigpiod_if2
  pthread
  rt
)

ament_target_dependencies(diffbot_gpio_node
  rclcpp
  sensor_msgs
  std_msgs
  rcl_interfaces
)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  TARGETS diffbot_gpio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
