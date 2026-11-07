cmake_minimum_required(VERSION 3.8)
project(roadwork_student)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roadwork_interfaces REQUIRED)
find_package(roadwork_student_interfaces REQUIRED)

add_executable(sense_node src/sense_node.cpp)
target_include_directories(sense_node PRIVATE include)
ament_target_dependencies(sense_node rclcpp builtin_interfaces nav_msgs sensor_msgs
  roadwork_interfaces roadwork_student_interfaces)

add_executable(plan_node src/plan_node.cpp)
target_include_directories(plan_node PRIVATE include)
ament_target_dependencies(plan_node rclcpp builtin_interfaces geometry_msgs nav_msgs std_msgs
  roadwork_student_interfaces)

add_executable(act_node src/act_node.cpp)
target_include_directories(act_node PRIVATE include)
ament_target_dependencies(act_node rclcpp builtin_interfaces roadwork_interfaces roadwork_student_interfaces)

install(TARGETS sense_node plan_node act_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(roadwork_core REQUIRED)
  foreach(t test_perception test_planning test_closed_loop)
    add_executable(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE include ${roadwork_core_INCLUDE_DIRS})
    ament_add_test(${t} GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 600 COMMAND $<TARGET_FILE:${t}>)
  endforeach()
endif()

ament_package()
