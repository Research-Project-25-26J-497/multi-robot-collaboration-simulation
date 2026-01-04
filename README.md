# Multi-Robot Collaborative Mapping System

A ROS 2 workspace for multi-robot simultaneous localization and mapping (SLAM) using Gazebo simulation and SLAM Toolbox.

## 📋 Overview

This project enables multiple robots to collaboratively explore and map an environment. Each robot runs its own SLAM instance, and their maps are merged into a unified representation for collaborative navigation and exploration.

### Key Features

- **Multi-Robot SLAM**: Independent SLAM instances for up to 4 robots
- **Map Merging**: Real-time fusion of individual robot maps into a unified global map
- **Gazebo Simulation**: Realistic warehouse environment for testing
- **Visualization**: RViz2 integration for monitoring multiple robot maps
- **Flexible Robot Generation**: Dynamic URDF generation with proper namespacing

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo Simulation                       │
│  ┌───────┐  ┌───────┐  ┌───────┐  ┌───────┐                │
│  │Robot1 │  │Robot2 │  │Robot3 │  │Robot4 │                │
│  └───┬───┘  └───┬───┘  └───┬───┘  └───┬───┘                │
└──────┼──────────┼──────────┼──────────┼─────────────────────┘
       │          │          │          │
       │/scan     │/scan     │/scan     │/scan
       │/odom     │/odom     │/odom     │/odom
       ▼          ▼          ▼          ▼
┌──────────────────────────────────────────────────────────────┐
│              SLAM Toolbox Nodes (per robot)                  │
│  ┌──────────────────────────────────────────────────┐        │
│  │ robot1/slam  robot2/slam  robot3/slam  robot4/slam│        │
│  └────┬──────────────┬──────────────┬─────────┬─────┘        │
└───────┼──────────────┼──────────────┼─────────┼──────────────┘
        │/map          │/map          │/map     │/map
        │              │              │         │
        └──────────────┴──────────────┴─────────┘
                       │
                       ▼
        ┌──────────────────────────┐
        │   Map Merger Node        │
        │  Combines individual     │
        │  robot maps using TF     │
        └──────────┬───────────────┘
                   │/merged_map
                   ▼
        ┌──────────────────────────┐
        │   RViz2 Visualization    │
        │  - Individual maps       │
        │  - Merged map           │
        │  - Robot positions       │
        └──────────────────────────┘
```

## 🚀 Quick Start

### Prerequisites

- **ROS 2 Humble** (or compatible distribution)
- **Gazebo** (gazebo_ros packages)
- **SLAM Toolbox**
- **RViz2**
- **Python 3**

### Installation

1. **Clone and build the workspace:**
```bash
cd ~/multi_robot_ws
colcon build
source install/setup.bash
```

2. **Make scripts executable:**
```bash
chmod +x generate_robot_urdf.py
chmod +x spawn_robots.py
chmod +x launch_multimap.sh
chmod +x save_map.sh
```

## 📖 Workflow

### Step 1: Launch Multi-Robot Mapping

The main launch script automates the entire setup process:

```bash
./launch_multimap.sh
```

**What happens internally:**

1. **Gazebo Launch**: Starts Gazebo with the warehouse world
   - World file: `src/worlds/multi_room_warehouse.world`
   - Empty simulation environment ready for robot spawning

2. **Robot Generation & Spawning**: For each robot (1-4):
   - Generates namespaced URDF using `generate_robot_urdf.py`
   - Creates proper TF frames (`robot1/base_link`, `robot1/odom`, etc.)
   - Spawns robot in Gazebo at designated position
   - Starts `robot_state_publisher` for TF broadcasting

3. **SLAM Initialization**: For each robot:
   - Launches `async_slam_toolbox_node` in robot's namespace
   - Configures parameters from `config/mapper_params_online_async.yaml`
   - Maps sensor data (`/robotN/scan`) to occupancy grids (`/robotN/map`)
   - Publishes transforms in robot's local frame

4. **Map Merging**:
   - Starts `map_merger_node` 
   - Subscribes to all robot map topics (`/robot1/map`, `/robot2/map`, etc.)
   - Uses TF transforms to align maps in global coordinate frame
   - Publishes unified `/merged_map` at 1 Hz

5. **Visualization**:
   - Launches RViz2 with custom configuration
   - Displays individual robot maps and merged result
   - Shows robot positions and trajectories

### Step 2: Monitor and Control

**View Topics:**
```bash
# List all active topics
ros2 topic list

# View specific robot's map
ros2 topic echo /robot1/map

# View merged map
ros2 topic echo /merged_map
```

**Check Transforms:**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map robot1/base_link
```

**Monitor SLAM Performance:**
```bash
# Watch a specific robot's SLAM node
ros2 node info /robot1/slam_toolbox
```

### Step 3: Save Maps

When mapping is complete, save the merged map:

```bash
./save_map.sh
```

**Interactive prompts:**
- Select map topic (default: `/merged_map`)
- Choose filename (default: `fused_warehouse_map`)

**Output:**
- `map_images/<name>.pgm` - Occupancy grid image
- `map_images/<name>.yaml` - Map metadata (resolution, origin, thresholds)

### Alternative: Manual Launch

For more control, launch components separately:

```bash
# Terminal 1: Gazebo
ros2 launch gazebo_ros gazebo.launch.py world:=~/multi_robot_ws/src/worlds/multi_room_warehouse.world

# Terminal 2: Spawn robots
./spawn_robots.py --count 4 --pattern grid

# Terminal 3: SLAM and mapping
ros2 launch multi_robot_collab multi_robot_mapping.launch.py

# Terminal 4: Save map when done
./save_map.sh
```

## 📁 Project Structure

```
multi_robot_ws/
├── src/
│   ├── multi_robot_collab/          # Main ROS 2 package
│   │   ├── config/
│   │   │   ├── mapper_params_online_async.yaml  # SLAM Toolbox config
│   │   │   ├── multi_robot_mapping.rviz         # RViz config
│   │   │   └── single_robot_mapping.rviz
│   │   ├── launch/
│   │   │   └── multi_robot_mapping.launch.py    # Main launch file
│   │   ├── multi_robot_collab/
│   │   │   ├── map_merger_node.py               # Map fusion logic
│   │   │   ├── map_fusion_node.py
│   │   │   ├── scan_merger_node.py
│   │   │   ├── unified_slam_node.py
│   │   │   └── advanced_collaboration_node.py
│   │   ├── worlds/
│   │   │   └── multi_room_warehouse.world       # Gazebo world
│   │   ├── package.xml
│   │   └── setup.py
│   ├── launch/
│   │   └── spawn_and_state.py
│   └── simple_robot_with_control.urdf           # Base robot URDF
├── generate_robot_urdf.py           # Generates namespaced URDFs
├── spawn_robots.py                  # Spawns multiple robots
├── launch_multimap.sh               # Main launch script
├── save_map.sh                      # Map saving utility
├── map_images/                      # Saved maps directory
├── build/                           # Build artifacts
├── install/                         # Installed packages
└── log/                            # Build logs
```

## 🔧 Configuration

### SLAM Parameters

Edit `src/multi_robot_collab/config/mapper_params_online_async.yaml` to tune:
- Map resolution
- Scan matching parameters
- Loop closure detection
- Optimization frequency

### Robot Count and Positions

**Modify in `launch_multimap.sh`:**
```bash
NUM_ROBOTS=4  # Change number of robots
```

**Or use `spawn_robots.py` with arguments:**
```bash
./spawn_robots.py --count 5 --spacing 3.0 --pattern grid
```

### Map Merger Settings

Edit `src/multi_robot_collab/multi_robot_collab/map_merger_node.py`:
```python
self.merged_map.info.resolution = 0.05  # 5cm cells
self.merged_map.info.width = 2000       # 100m x 100m
self.merged_map.info.height = 2000
```

## 🤖 Robot Design

Each robot has:
- **Differential drive base** (2 wheels + caster)
- **LiDAR sensor** (360° laser scanner)
- **Dimensions**: 0.3m × 0.3m × 0.1m base
- **Namespace**: All components prefixed with robot name

**Generated URDF includes:**
- `<robot_name>/base_link`
- `<robot_name>/left_wheel`, `<robot_name>/right_wheel`
- `<robot_name>/caster`
- `<robot_name>/laser`
- `<robot_name>/odom` frame

## 🎯 Use Cases

1. **Warehouse Mapping**: Multiple robots map large warehouse spaces
2. **Search & Rescue**: Collaborative exploration of unknown environments
3. **Multi-Floor Buildings**: Different robots map different floors
4. **Research**: Study multi-robot coordination and map fusion algorithms

## 🔍 Troubleshooting

### Robots not spawning
- Check Gazebo is fully loaded before spawning
- Verify URDF paths in scripts
- Ensure proper ROS 2 sourcing

### SLAM not starting
- Verify `/scan` and `/odom` topics exist for each robot
- Check SLAM Toolbox installation
- Review namespace configuration

### Maps not merging
- Confirm all robot maps publishing on `/robotN/map`
- Check TF transforms between robots
- Verify `map_merger_node` is running

### RViz crashes
- Reduce number of displayed maps
- Check RViz config file paths
- Ensure sufficient system memory

## 📊 Performance Tips

- **Reduce map size** for faster processing
- **Adjust SLAM frequency** in config for lower CPU usage
- **Limit active robots** based on hardware capabilities
- **Use ROSbag** to record and replay experiments

## 📝 Development

### Adding New Nodes

1. Create Python file in `src/multi_robot_collab/multi_robot_collab/`
2. Add entry point in `setup.py`
3. Rebuild workspace: `colcon build`

### Modifying Launch Files

Edit `src/multi_robot_collab/launch/multi_robot_mapping.launch.py` to:
- Add/remove robots
- Include additional nodes
- Modify parameters

### Custom Worlds

Place `.world` files in `src/multi_robot_collab/worlds/` and reference in launch scripts.

## 📚 Dependencies

**ROS 2 Packages:**
- `rclpy` - Python client library
- `nav_msgs` - Navigation messages (OccupancyGrid)
- `sensor_msgs` - Sensor messages (LaserScan)
- `tf2_ros` - Transform library
- `slam_toolbox` - SLAM implementation
- `gazebo_ros` - Gazebo integration

**System Requirements:**
- Ubuntu 22.04 (recommended)
- 8GB+ RAM (for 4 robots)
- Multi-core CPU

**Last Updated**: January 2026
