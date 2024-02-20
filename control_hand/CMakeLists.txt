
cmake_minimum_required(VERSION 3.8)
project(control_hand)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})

Set (AllegroHand_VERSION_MAJOR 1)
Set (AllegroHand_VERSION_MINOR 0)

set (CMAKE_CXX_FLAGS "-O2 -Wall")

add_library(
  control_hand
  src/canAPI.cpp
  src/can_communicator.cpp
  src/RockScissorsPaper.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(controller src/controller.cpp)
add_executable(moveit_controller src/moveit_controller.cpp)

ament_target_dependencies(controller rclcpp std_msgs std_srvs)
ament_target_dependencies(moveit_controller rclcpp sensor_msgs std_msgs std_srvs moveit_ros_planning_interface moveit_core moveit_ros_planning)

target_include_directories(control_hand
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
"srv/Move.srv"
"srv/SetJoints.srv"
LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces
  "rosidl_typesupport_cpp"
)



install(DIRECTORY include/control_hand DESTINATION include)

target_link_libraries(controller control_hand BHand pcanbasic pthread)
target_link_libraries(moveit_controller control_hand BHand pcanbasic pthread)
target_link_libraries(control_hand "${cpp_typesupport_target}")


  
install(TARGETS 
controller
DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
moveit_controller
DESTINATION lib/${PROJECT_NAME})

install(TARGETS control_hand
  EXPORT control_hand-targets
  DESTINATION lib/${PROJECT_NAME})

  install(EXPORT control_hand-targets
  FILE control_hand-targets.cmake
  NAMESPACE control_hand::
  DESTINATION lib/cmake/${PROJECT_NAME}) 

install(DIRECTORY
  srv
  meshes
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)




ament_package()
