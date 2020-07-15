cmake_minimum_required(VERSION 2.8.3)
project(sc_hw)

find_package(catkin REQUIRED COMPONENTS
        controller_manager
        geometry_msgs
        sensor_msgs
        hardware_interface
        realtime_tools
        roscpp
        std_msgs
        sc_msgs
        tf
)

set(config_path \"${CMAKE_CURRENT_SOURCE_DIR}\")
add_definitions(-DCONFIG_PATH=${config_path})
catkin_package(   
  INCLUDE_DIRS include
#  LIBRARIES sc_hw
#  CATKIN_DEPENDS controller_manager geometry_msgs hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(${catkin_INCLUDE_DIRS})

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  include/sdk/sc_link/
  include/sdk/robot_abstract/
)

add_executable(sc_hw_node src/main.cpp src/transport_serial.cpp include/sdk/sc_link/sc_link.cpp include/sdk/sc_link/sc_link_state_machine.cpp src/sc_hw.cpp src/sc_hw_ros.cpp)

add_dependencies(sc_hw_node sc_msgs_gencpp)

target_link_libraries(sc_hw_node ${catkin_LIBRARIES})


