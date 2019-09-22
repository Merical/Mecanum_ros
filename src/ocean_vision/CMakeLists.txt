cmake_minimum_required(VERSION 3.0)
project(ocean_vision)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation image_transport cv_bridge)
find_package(OpenCV 3.4.0 REQUIRED)

add_message_files(FILES Cmt.msg)
generate_messages(DEPENDENCIES std_msgs)

add_definitions ("-Wall -lboost_regex")
set(CMAKE_CXX_STANDARD 11)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        )

include_directories("./include" ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable (cmt src/common.cpp src/gui.cpp src/main.cpp
    src/CMT.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp
    src/fastcluster.cpp)
target_link_libraries(cmt ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(cmt ocean_vision_generate_messages_cpp)

add_executable (realsense_image src/realsense_image.cpp)
target_link_libraries(realsense_image ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(realsense_image ocean_vision_generate_messages_cpp)

add_executable (cmt_ros src/common.cpp src/gui.cpp src/cmt_ros.cpp
    src/CMT.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp
    src/fastcluster.cpp)
target_link_libraries(cmt_ros ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(cmt_ros ocean_vision_generate_messages_cpp)

add_executable (cmt_ros_server src/common.cpp src/gui.cpp src/cmt_ros_server.cpp
        src/CMT.cpp src/Consensus.cpp src/Fusion.cpp src/Matcher.cpp src/Tracker.cpp
        src/fastcluster.cpp)
target_link_libraries(cmt_ros_server ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(cmt_ros_server ocean_vision_generate_messages_cpp)

add_executable (ros_rgbd src/ros_rgbd.cpp)
target_link_libraries(ros_rgbd ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_rgbd ocean_vision_generate_messages_cpp)
