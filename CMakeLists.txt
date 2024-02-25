cmake_minimum_required(VERSION 3.5)
project(realsense_rgbd_transport_ros2)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(compressed_image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rgbd_pub src/rgbd_pub.cpp)
target_link_libraries(rgbd_pub ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgbd_pub rclcpp std_msgs realsense2 OpenCV image_transport cv_bridge)

add_executable(rgbd_pub_aligned src/rgbd_pub_aligned.cpp)
target_link_libraries(rgbd_pub_aligned ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgbd_pub_aligned rclcpp std_msgs realsense2 OpenCV image_transport cv_bridge)

add_executable(rgbd_sub src/rgbd_sub.cpp)
target_link_libraries(rgbd_sub ${OpenCV_LIBS})
ament_target_dependencies(rgbd_sub rclcpp std_msgs OpenCV image_transport cv_bridge)

add_executable(rgbd_sub_compressed src/rgbd_sub_compressed.cpp)
target_link_libraries(rgbd_sub_compressed ${OpenCV_LIBS})
ament_target_dependencies(rgbd_sub_compressed rclcpp std_msgs OpenCV image_transport cv_bridge)

install(TARGETS
        rgbd_pub
        rgbd_pub_aligned
        rgbd_sub
        rgbd_sub_compressed
        DESTINATION lib/${PROJECT_NAME})

ament_package()