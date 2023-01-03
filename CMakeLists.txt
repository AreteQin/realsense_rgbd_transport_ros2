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
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(publisher rclcpp std_msgs realsense2 OpenCV image_transport cv_bridge)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${OpenCV_LIBS} ${realsense2_LIBRARY})
ament_target_dependencies(subscriber rclcpp std_msgs realsense2 OpenCV image_transport cv_bridge)

install(TARGETS
        publisher
        subscriber
        DESTINATION lib/${PROJECT_NAME})

ament_package()