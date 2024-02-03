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
find_package(compressed_image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rgbd_pub src/rgbd_pub.cpp)
target_link_libraries(rgbd_pub ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgbd_pub rclcpp std_msgs realsense2 OpenCV image_transport cv_bridge)

add_executable(rgbd_sub src/rgbd_sub.cpp)
target_link_libraries(rgbd_sub ${OpenCV_LIBS})
ament_target_dependencies(rgbd_sub rclcpp std_msgs OpenCV image_transport cv_bridge)

add_executable(rgbd_pub_compressed src/rgbd_pub_compressed.cpp)
target_link_libraries(rgbd_pub_compressed ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgbd_pub_compressed rclcpp std_msgs realsense2 OpenCV image_transport cv_bridge)

add_executable(rgbd_sub_compressed src/rgbd_sub_compressed.cpp)
target_link_libraries(rgbd_sub_compressed ${OpenCV_LIBS})
ament_target_dependencies(rgbd_sub_compressed rclcpp std_msgs OpenCV image_transport cv_bridge)

add_executable(rgb_pub src/rgb_pub.cpp)
target_link_libraries(rgb_pub ${OpenCV_LIBS})
ament_target_dependencies(rgb_pub rclcpp std_msgs OpenCV image_transport cv_bridge)

add_executable(rgb_sub src/rgb_sub.cpp)
target_link_libraries(rgb_sub ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(rgb_sub rclcpp std_msgs OpenCV image_transport cv_bridge)

find_package(std_msgs REQUIRED)
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS
        talker
        DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS
        listener
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        rgbd_pub
        rgbd_sub
        rgb_pub
        rgb_sub
        rgbd_pub_compressed
        rgbd_sub_compressed
        DESTINATION lib/${PROJECT_NAME})

ament_package()