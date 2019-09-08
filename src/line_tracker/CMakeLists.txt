cmake_minimum_required(VERSION 2.8.3)
project(line_tracker)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation image_transport cv_bridge sc_msgs)
find_package(OpenCV 3.4.0 REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

add_definitions ("-Wall -lboost_regex")
set(CMAKE_CXX_STANDARD 11)

catkin_package(
        CATKIN_DEPENDS
        )

include_directories("./include/line_tracker" ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable (line_tracker_node src/line_ros.cpp src/line_tracker.cpp)
target_link_libraries(line_tracker_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(line_tracker_node sc_msgs_gencpp)