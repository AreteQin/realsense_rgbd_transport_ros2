# realsense_rgbd_transport_ros2

## Dependencies
```bash
sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

## Usage
```bash
ros2 run realsense_rgbd_transport_ros2 rgbd_sub --ros-args --remap _image_transport:=compressed
```

## Performance
|Version|Platform|WiFi|BandWidth|Latency|
|-|-|-|-|-|
|ROS1 Noetic|Ubuntu 18.04|5G|~10Mbps|~0.1s|