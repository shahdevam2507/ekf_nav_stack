cmake_minimum_required(VERSION 3.8)
project(teleop)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)



add_executable(teleop_ps4 src/teleop.cpp)
add_executable(teleop_sim src/teleop_sim.cpp)


ament_target_dependencies(teleop_ps4 rclcpp geometry_msgs sensor_msgs)
ament_target_dependencies(teleop_sim rclcpp geometry_msgs sensor_msgs)

install(TARGETS
 teleop_ps4
 teleop_sim
    DESTINATION lib/${PROJECT_NAME})

ament_package()