# realsense_rgbd_transport_ros2

## Dependencies
```bash
sudo apt install ros-${ROS_DISTRO}-compressed-image-transport
```

## Usage
```bash
ros2 run realsense_rgbd_transport_ros2 rgbd_pub
ros2 run realsense_rgbd_transport_ros2 rgbd_sub
ros2 run realsense_rgbd_transport_ros2 rgbd_sub_compressed
```

## Performance
| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 5G   | No         | 42Mbps    | 0.115s  | 24        |
| ROS2 Rolling | Ubuntu 22.04 | 5G   | Yes        | 4.2Mbps   | 0.1s    | 29.9      |
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | No         | 4.2Mbps   | 0.15s   | 3         |
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | Yes        | 4Mbps #   | 0.07s # | 13 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | No         | 30Mbps #  | 0.5s    | 20 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | Yes        | 1.27Mbps  | 0.53s   | 20        |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | No         | 4.8Mbps # | 1s #    | 3 #       |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | Yes        | 1.26Mbps  | 0.688s  | 15.7      |
| ROS2 foxy | Ubuntu 20.04 | 5G   | No            |   35.19  | 0.055s  |    23.1     |
| ROS2 foxy | Ubuntu 20.04 | 5G   | Yes           |  |     |      |
| ROS2 foxy | Ubuntu 20.04 | 2.4G | No            |    |    |          |
| ROS2 foxy | Ubuntu 20.04 | 2.4G | Yes           |   |  |       |
| foxy (bridge) and noetic | Ubuntu 20.04 |5G|No  |   28.87 | 0.327s  |    21.5     |
| ROS2 foxy | Ubuntu 20.04 | 5G   | Yes           | 3.265 |  0.07   |   29.99   |
| ROS2 foxy | Ubuntu 20.04 | 2.4G | No            |    |    |          |
| ROS2 foxy | Ubuntu 20.04 | 2.4G | Yes           |   |  |       |

\# means the value is very unstable.

## Performance
| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 5G   | No         | 42Mbps    | 0.115s  | 24        |
| ROS2 Rolling | Ubuntu 22.04 | 5G   | Yes        | 4.2Mbps   | 0.1s    | 29.9      |
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | No         | 4.2Mbps   | 0.15s   | 3         |
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | Yes        | 4Mbps #   | 0.07s # | 13 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | No         | 30Mbps #  | 0.5s    | 20 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | Yes        | 1.27Mbps  | 0.53s   | 20        |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | No         | 4.8Mbps # | 1s #    | 3 #       |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | Yes        | 1.26Mbps  | 0.688s  | 15.7      |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 5G   | No         | 42Mbps    | 0.115s  | 24        |
| ROS2 Rolling | Ubuntu 22.04 | 5G   | Yes        | 4.2Mbps   | 0.1s    | 29.9      |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | No         | 4.2Mbps   | 0.15s   | 3         |
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | Yes        | 4Mbps #   | 0.07s # | 13 #      |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 5G   | No         | 42Mbps    | 0.115s  | 24        |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | No         | 30Mbps #  | 0.5s    | 20 #      |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 5G   | Yes        | 4.2Mbps   | 0.1s    | 29.9      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | Yes        | 1.27Mbps  | 0.53s   | 20        |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | No         | 4.2Mbps   | 0.15s   | 3         |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | No         | 4.8Mbps # | 1s #    | 3 #       |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | Yes        | 4Mbps #   | 0.07s # | 13 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | No         | 4.8Mbps # | 1s #    | 3 #       |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | Yes        | 1.26Mbps  | 0.688s  | 15.7      |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 5G   | No         | 42Mbps    | 0.115s  | 24        |
| ROS2 Rolling | Ubuntu 22.04 | 5G   | Yes        | 4.2Mbps   | 0.1s    | 29.9      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | No         | 30Mbps #  | 0.5s    | 20 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 5G   | Yes        | 1.27Mbps  | 0.53s   | 20        |

\# means the value is very unstable.

| Version      | Platform     | WiFi | Compressed | BandWidth | Latency | Frequency |
|--------------|--------------|------|------------|-----------|---------|-----------|
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | No         | 4.2Mbps   | 0.15s   | 3         |
| ROS2 Rolling | Ubuntu 22.04 | 2.4G | Yes        | 4Mbps #   | 0.07s # | 13 #      |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | No         | 4.8Mbps # | 1s #    | 3 #       |
| ROS1 Noetic  | Ubuntu 20.04 | 2.4G | Yes        | 1.26Mbps  | 0.688s  | 15.7      |

\# means the value is very unstable.
