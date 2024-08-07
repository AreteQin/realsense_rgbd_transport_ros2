cmake_minimum_required(VERSION 3.5)
project(realsense_rgbd_transport_ros2)

##Default to C++14
#if (NOT CMAKE_CXX_STANDARD)
#    set(CMAKE_CXX_STANDARD 14)
#endif ()
#
#if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif ()

# For ROS2 Foxy
# Check the ROS distribution
execute_process(
        COMMAND bash -c "echo $ROS_DISTRO"
        OUTPUT_VARIABLE ROS_DISTRO
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
# Print the ROS distribution (optional, for debugging)
message(STATUS "ROS Distribution: ${ROS_DISTRO}")
if (ROS_DISTRO STREQUAL "foxy")
    message(STATUS "Detected ROS 2 Foxy")
    add_definitions(-DROS2_DISTRO_FOXY)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(rgbd_pub src/rgbd_pub.cpp)
target_link_libraries(rgbd_pub ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgbd_pub
        rclcpp
        std_msgs
        realsense2
        OpenCV
        image_transport
        compressed_image_transport
        cv_bridge)

add_executable(rgbd_pub_aligned src/rgbd_pub_aligned.cpp)
target_link_libraries(rgbd_pub_aligned ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgbd_pub_aligned
        rclcpp
        std_msgs
        realsense2
        OpenCV
        compressed_image_transport
        image_transport
        cv_bridge)

add_executable(rgbd_sub src/rgbd_sub.cpp)
target_link_libraries(rgbd_sub ${OpenCV_LIBS})
ament_target_dependencies(rgbd_sub rclcpp std_msgs OpenCV image_transport cv_bridge)

add_executable(rgbd_sub_compressed src/rgbd_sub_compressed.cpp)
target_link_libraries(rgbd_sub_compressed ${OpenCV_LIBS})
ament_target_dependencies(rgbd_sub_compressed
        rclcpp
        std_msgs
        OpenCV
        image_transport
        cv_bridge
        compressed_image_transport
)

add_executable(qcar_sub src/qcar_sub.cpp)
target_link_libraries(qcar_sub ${OpenCV_LIBS})
ament_target_dependencies(qcar_sub
        rclcpp
        std_msgs
        OpenCV
        image_transport
        cv_bridge
        compressed_image_transport
)

install(TARGETS
        rgbd_pub
        rgbd_pub_aligned
        rgbd_sub
        rgbd_sub_compressed
        qcar_sub
        DESTINATION lib/${PROJECT_NAME})

ament_package()
