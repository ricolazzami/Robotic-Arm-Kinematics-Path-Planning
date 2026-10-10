cmake_minimum_required(VERSION 3.8)
project(arm_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trac_ik_lib REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  arm_planning/ik_solver_node.py
  arm_planning/trajectory_planner_node.py
  arm_planning/collision_checker_node.py
  arm_planning/planning_coordinator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ik_solver_cpp src/ik_solver.cpp)
target_include_directories(ik_solver_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ik_solver_cpp PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ik_solver_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  trac_ik_lib
)

install(TARGETS ik_solver_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
