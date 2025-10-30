Complete obstacle avoidance implementation for TurtleBot3 Waffle in Gazebo simulation.

## 🤖 Overview

This package enables autonomous navigation for TurtleBot3 with:
- ✅ Real LIDAR data processing (360° coverage)
- ✅ Intelligent obstacle detection and avoidance
- ✅ Smart turning (chooses direction with more space)
- ✅ Real-time status logging to file and console
- ✅ Dynamic parameter reconfiguration
- ✅ Comprehensive unit tests

## 🏗️ Architecture

### Nodes

1. **obstacle_avoider**
   - Subscribes to `/scan` (LIDAR data from TurtleBot3)
   - Publishes to `/cmd_vel` (velocity commands)
   - Publishes to `/robot_status` (state information)
   - Analyzes front, left, and right sectors
   - Intelligently decides turn direction

2. **status_logger** (Extra Feature)
   - Subscribes to `/robot_status` and `/cmd_vel`
   - Logs data to CSV file in `~/ros2_logs/`
   - Provides real-time console output
   - Tracks all sensor readings and commands

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LIDAR data (input) |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (output) |
| `/robot_status` | std_msgs/String | Robot state info |

### Parameters

Configured in `config/params.yaml`:

```yaml
obstacle_avoider:
  safety_distance: 0.5      # Minimum safe distance (meters)
  linear_speed: 0.22        # Forward speed (m/s)
  angular_speed: 1.5        # Turning speed (rad/s)
  front_angle_range: 45.0   # Front detection sector (degrees)
  side_angle_range: 30.0    # Side detection sectors (degrees)

status_logger:
  log_to_file: False       # Change it  if you want to log the file
  log_file: 'turtlebot3_status.log'
  log_directory: '~/ros2_logs'
```

## 🚀 Installation & Setup

### Prerequisites

- ROS 2 Humble
- TurtleBot3 packages
- Gazebo
- Python 3.8+

### Install TurtleBot3 (if not already installed)

```bash
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Build the Package

```bash
# Navigate to workspace
cd ~/ros2_ws/src/ros2_obstacle_avoidance

# Return to workspace root
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select ros2_obstacle_avoidance

# Source
source install/setup.bash
```

## 🎮 Running the System

### Complete Launch (Recommended)

**Terminal 1: Launch TurtleBot3 Gazebo World**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2: Launch Obstacle Avoidance**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ros2_obstacle_avoidance obstacle_avoidance_launch.launch.py
```

### Run Individual Nodes Instead of launch file for  Obstacle avoidance 

```bash
# Terminal 1: Obstacle avoider
ros2 run ros2_obstacle_avoidance obstacle_avoider

# Terminal 2: Status logger
ros2 run ros2_obstacle_avoidance status_logger
```

### Monitor & Debug

```bash
# View LIDAR data
ros2 topic echo /scan

# View velocity commands
ros2 topic echo /cmd_vel

# View robot status
ros2 topic echo /robot_status

# List all active topics
ros2 topic list

# Check node info
ros2 node info /obstacle_avoider
```

### Dynamic Parameter Adjustment

```bash
# List current parameters
ros2 param list /obstacle_avoider

# Change safety distance
ros2 param set /obstacle_avoider safety_distance 0.7

# Change speed
ros2 param set /obstacle_avoider linear_speed 0.15

# Change turning speed
ros2 param set /obstacle_avoider angular_speed 1.0
```

## 🧪 Testing

### Run Unit Tests

```bash

cd ~/ros2_ws/src/ros2_obstacle_avoidance
python3 -m pytest test/test_obstacle_avoider.py -v
or 
python3 -m pytest test/test_obstacle_avoider.py -v -s --tb=short

```

### Manual Testing Scenarios

1. **Basic Avoidance**: Let robot navigate automatically
2. **Speed Adjustment**: Change speeds during operation
3. **Safety Distance**: Modify threshold and observe behavior
4. **Log Analysis**: Check `~/ros2_logs/turtlebot3_status.log`

## 📊 Extra Feature: Status Logger

The status logger provides comprehensive monitoring:
To Log the file change the log_to_file parameter to True default is false 
**Console Output:**
```
2025-10-29 19:22:41.608, AVOIDING_OBSTACLE, 0.49, 2.11, 0.43, 0.000, 1.500
2025-10-29 19:22:41.810, MOVING_FORWARD,    1.08, 1.17, 0.41, 0.220, 0.000
```
🎥 Demo Video

Watch the full obstacle avoidance demo here:
👉 TurtleBot3 Obstacle Avoidance - https://youtu.be/1rJc55SlY98
