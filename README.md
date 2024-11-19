# TurtleBot3 Simulation Setup with ROS2 Humble

This repository contains the setup and launch files for simulating TurtleBot3 robots in different Gazebo environments using ROS2 Humble. It includes configurations for simulation, SLAM, and navigation.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble (full desktop installation)
- Gazebo 11.x
- Git

## Installation

### 1. ROS2 Humble Installation
Ensure you have ROS2 Humble installed. If not, follow the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

### 2. Create Workspace and Clone Repository

```bash
# Create workspace directory
mkdir -p ~/turtlebot3-ros2humble-simulations/src
cd ~/turtlebot3-ros2humble-simulations/src

# Clone this repository
git clone https://github.com/prakash-aryan/turtlebot3-ros2humble-simulations.git .

# Clone required repositories
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

### 3. Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-dynamixel-sdk \
    ros-humble-tf2-ros \
    ros-humble-tf2-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ament-cmake \
    ros-humble-urdf \
    ros-humble-rclpy \
    ros-humble-gazebo-ros2-control \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher

# Initialize rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace

```bash
cd ~/turtlebot3-ros2humble-simulations
colcon build --symlink-install
```

## Basic Usage

### 1. Source the Workspace
Before running any commands, source your workspace in each new terminal:

```bash
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
```

### 2. Set Environment Variables
Set the TurtleBot3 model (required for all launches):

```bash
export TURTLEBOT3_MODEL=burger
```

### 3. Launch Simulation

#### Empty World
```bash
export WORLD_NAME=empty_world
ros2 launch custom_launch custom_turtlebot3.launch.py
```

#### TurtleBot3 House (recommended for SLAM and Navigation)
```bash
export WORLD_NAME=turtlebot3_house
ros2 launch custom_launch custom_turtlebot3.launch.py
```

#### TurtleBot3 World
```bash
export WORLD_NAME=turtlebot3_world
ros2 launch custom_launch custom_turtlebot3.launch.py
```

### 4. Basic Robot Control

#### Teleoperation
In a new terminal:
```bash
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Control keys:
- `w`: Forward
- `x`: Backward
- `a`: Rotate left
- `d`: Rotate right
- `s`: Stop
- `space`: Emergency stop

## SLAM (Simultaneous Localization and Mapping)

### 1. Launch Simulation with Appropriate World
```bash
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
export TURTLEBOT3_MODEL=burger
export WORLD_NAME=turtlebot3_world  # Better for SLAM
ros2 launch custom_launch custom_turtlebot3.launch.py
```

### 2. Launch Cartographer SLAM
In a new terminal:
```bash
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 3. Control Robot for Mapping
In another terminal:
```bash
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

Mapping controls:
- `w/x`: Increase/decrease linear velocity
- `a/d`: Increase/decrease angular velocity
- `space` or `s`: Stop
- `CTRL-C`: Quit

### 4. Save the Map
Once you've created a good map, save it in a new terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

This will save two files:
- `map.pgm`: The map image
- `map.yaml`: The map metadata

### Tips for Good Mapping
1. Move slowly and smoothly
2. Make sure to cover all areas
3. Try to close loops for better accuracy
4. Keep the environment well-lit
5. Avoid rapid rotations
6. Ensure good overlap between scans

## Navigation (Nav2)

### 1. Launch Navigation with Existing Map
After creating a map, you can use it for navigation:

```bash
# Terminal 1: Launch Simulation
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
export TURTLEBOT3_MODEL=burger
export WORLD_NAME=turtlebot3_world
ros2 launch custom_launch custom_turtlebot3.launch.py

# Terminal 2: Launch Navigation
source ~/turtlebot3-ros2humble-simulations/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup navigation_launch.py map:=/path/to/your/map.yaml
```

## Custom Launch File Configuration

The custom launch file (`custom_turtlebot3.launch.py`) supports various arguments:

- `world_name`: Select simulation world (`empty_world`, `turtlebot3_house`, `turtlebot3_world`)
- `x_pose`: Initial X position of the robot (default: -2.0)
- `y_pose`: Initial Y position of the robot (default: -0.5)

Example with custom position:
```bash
ros2 launch custom_launch custom_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0
```

## Troubleshooting

### 1. Simulation Issues
- If the robot flies off in simulation:
  - Try different spawn positions using `x_pose` and `y_pose` arguments
  - Use the empty world first to test stability
  - Check if the physics parameters are correct

### 2. SLAM Issues
- If the map appears distorted:
  - Ensure `use_sim_time:=True` is set
  - Move the robot more slowly
  - Check if all sensors are publishing data (`ros2 topic list`)
- If the map is not updating:
  - Verify laser scan data publication
  - Check TF transformations
  - Ensure proper odometry data

### 3. General Issues
- If launch files fail:
  - Ensure all dependencies are installed
  - Check if the TurtleBot3 model is set correctly
  - Verify that the workspace is properly sourced
- If packages are not found:
  - Rebuild the workspace using `colcon build --symlink-install`
  - Source the workspace again

## Directory Structure
```
turtlebot3-ros2humble-simulations/
├── src/
│   ├── custom_launch/
│   ├── DynamixelSDK/
│   ├── turtlebot3/
│   ├── turtlebot3_msgs/
│   └── turtlebot3_simulations/
```

## Additional Resources

- [TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [Cartographer ROS Documentation](https://google-cartographer-ros.readthedocs.io/)
