# realsense_rgbd_transport_ros2

## Dependencies
```bash
sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

## Usage
```bash
ros2 run realsense_rgbd_transport_ros2 rgbd_pub
ros2 run realsense_rgbd_transport_ros2 rgbd_sub
ros2 run realsense_rgbd_transport_ros2 rgbd_pub_compressed
ros2 run realsense_rgbd_transport_ros2 rgbd_sub_compressed
```

## Performance
|Version|Platform|WiFi|BandWidth|Latency|
|-|-|-|-|-|
|ROS1 Noetic|Ubuntu 18.04|5G|~10Mbps|~0.1s|