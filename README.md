# ORB-SLAM3 ROS2 Humble for Clearpath Jackal Simulation

A complete ROS2 Humble package for running ORB-SLAM3 monocular SLAM with Clearpath Jackal robot in Gazebo simulation. Includes trajectory visualization and point cloud mapping.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![License](https://img.shields.io/badge/License-GPLv3-green.svg)

## Features

- ORB-SLAM3 Monocular SLAM integrated with ROS2 Humble
- Live camera feed from Clearpath Jackal simulation
- Real-time trajectory publishing (`/orb_slam3/trajectory`)
- Camera pose publishing (`/orb_slam3/camera_pose`)
- TF broadcasting for RViz visualization
- Point cloud mapping using LiDAR + SLAM poses
- Coordinate frame transformation (camera → ROS convention)
- **Docker support with persistent volumes** (your work is saved!)

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Docker & Docker Compose (recommended)
- NVIDIA GPU + nvidia-docker (for Pangolin visualization)
- Clearpath Simulator (for Jackal simulation)

## Installation

### Option 1: Docker (Recommended)

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/orb_slam3_jackal.git
cd orb_slam3_jackal

# Create data directories for persistence
mkdir -p data/output

# Allow X11 forwarding
xhost +local:docker

# Build Docker image (first time only, ~15-30 min)
docker compose build

# Run container
docker compose up -d
docker exec -it orb_slam3_jackal bash
```

#### Persistent Data with Docker

The docker-compose.yml is configured to persist your work:

| Volume | Purpose |
|--------|---------|
| `./` → `/root/clearpath_ws/src/ros2_orb_slam3` | Source code (on your host) |
| `orb_slam3_build` | Build artifacts (no rebuild!) |
| `orb_slam3_install` | Install space |
| `./data/output` → `/tmp` | Trajectories & maps saved here |

**Your trajectories are automatically saved to `./data/output/` on your host machine!**

#### Docker Commands

```bash
# Start container
docker compose up -d

# Enter container
docker exec -it orb_slam3_jackal bash

# Stop container (data persists!)
docker compose down

# Remove container AND volumes (deletes build cache!)
docker compose down -v

# Rebuild after Dockerfile changes
docker compose build --no-cache
```

### Option 2: Native Installation

```bash
# Create workspace
mkdir -p ~/clearpath_ws/src
cd ~/clearpath_ws/src

# Clone this repository
git clone https://github.com/YOUR_USERNAME/orb_slam3_jackal.git ros2_orb_slam3

# Install Pangolin
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build -j$(nproc)
sudo cmake --install build

# Add to library path
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Install dependencies
cd ~/clearpath_ws
sudo apt update
sudo apt install -y libeigen3-dev libopencv-dev libboost-all-dev libssl-dev \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-tf2-ros \
    ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-topic-tools

# Build
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install

# Setup symlink for config path
mkdir -p ~/ros2_test/src
ln -s ~/clearpath_ws/src/ros2_orb_slam3 ~/ros2_test/src/ros2_orb_slam3
```

## Usage

### 1. Start the Simulation (on host or separate terminal)

```bash
ros2 launch clearpath_gz simulation.launch.py world:=marsyard
```

### 2. Setup TF Relays (required for RViz)

```bash
# Terminal 1 - TF setup
ros2 run topic_tools relay /j100_0000/tf /tf &
ros2 run topic_tools relay /j100_0000/tf_static /tf_static &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0.15 0 0 0 base_link lidar3d_0_laser &
```

### 3. Start ORB-SLAM3

```bash
# Terminal 2 - C++ SLAM node
cd ~/clearpath_ws && source install/setup.bash
ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp
```

```bash
# Terminal 3 - Python camera driver
cd ~/clearpath_ws && source install/setup.bash
python3 ~/clearpath_ws/src/ros2_orb_slam3/scripts/mono_live_driver.py \
    --ros-args -p settings_name:=Jackal_Sim \
    -p camera_topic:=/j100_0000/sensors/camera_0/color/image
```

### 4. Point Cloud Mapping (optional)

```bash
# Terminal 4 - Point cloud mapper
cd ~/clearpath_ws && source install/setup.bash
python3 ~/clearpath_ws/src/ros2_orb_slam3/scripts/pointcloud_mapper.py
```

### 5. Visualization

```bash
# Terminal 5 - RViz
rviz2 -d ~/clearpath_ws/src/ros2_orb_slam3/config/orb_slam3.rviz
```

### 6. Teleoperation

```bash
# Terminal 6 - Drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/j100_0000/cmd_vel
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/orb_slam3/camera_pose` | `geometry_msgs/PoseStamped` | Current camera pose in map frame |
| `/orb_slam3/trajectory` | `nav_msgs/Path` | Full trajectory path |
| `/point_cloud_map` | `sensor_msgs/PointCloud2` | Accumulated point cloud map |

## TF Frames

```
map
 └── odom
      └── base_link
           └── lidar3d_0_laser
 └── camera_link (from ORB-SLAM3)
```

## Configuration

### Camera Settings

Edit `orb_slam3/config/Monocular/Jackal_Sim.yaml` to adjust camera parameters:

```yaml
Camera.fx: 443.5300081599877
Camera.fy: 443.53000815998763
Camera.cx: 320.0
Camera.cy: 240.0
Camera.width: 640
Camera.height: 480
Camera.fps: 30.0
```

### ORB Extractor Settings

```yaml
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

## Troubleshooting

### TF frame not found
Make sure to run the TF relay commands before starting RViz.

### ORB-SLAM3 not initializing
Move the robot around to provide enough visual features for initialization.

### Point cloud not aligned
Check that all TF transforms are properly connected using:
```bash
ros2 run tf2_ros tf2_echo map lidar3d_0_laser
```

## File Structure

```
ros2_orb_slam3/
├── CMakeLists.txt
├── package.xml
├── Dockerfile
├── docker-compose.yml
├── README.md
├── config/
│   └── orb_slam3.rviz
├── include/ros2_orb_slam3/
│   └── common.hpp
├── src/
│   ├── mono_example.cpp
│   └── common.cpp
├── scripts/
│   ├── mono_live_driver.py
│   ├── pointcloud_mapper.py
│   └── plot_trajectory.py
├── orb_slam3/
│   ├── config/Monocular/
│   │   └── Jackal_Sim.yaml
│   ├── Vocabulary/
│   │   └── ORBvoc.txt.bin
│   ├── include/
│   ├── src/
│   └── Thirdparty/
│       ├── DBoW2/
│       ├── g2o/
│       └── Sophus/
└── ros2_orb_slam3/
    └── __init__.py
```

## Credits

- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) by Carlos Campos et al.
- [ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) by Azmyin Md. Kamal
- [Clearpath Robotics](https://github.com/clearpathrobotics) for the Jackal simulation

## License

This project is licensed under the GPLv3 License - see the [LICENSE](LICENSE) file for details.

## Citation

If you use this work, please cite:

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\'omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
}
```
