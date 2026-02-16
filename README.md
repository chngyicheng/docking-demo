# Robust Docking with LiDAR-Based Pose Refinement

Precision docking system for TurtleBot3 that combines Nav2 coarse navigation with RANSAC-based LiDAR wall detection for final approach control. The system does not trust SLAM localization in the final ~1m — instead, it estimates docking target geometry directly from laser scans and drives a closed-loop control law to converge to the desired relative pose.

Tested under scan dropout and scan corruption disturbances.

## Architecture Overview

```
┌─────────────┐    /goal_pose    ┌──────┐    /cmd_vel     ┌───────────┐
│ DockingNode │ ──────────────── │ Nav2 │ ─────────────── │ TurtleBot │
│             │    (IDLE phase)  └──────┘                 └───────────┘
│             │                                                   │
│  RANSAC     │◄── /scan_disturbed ◄── DisturbanceNode ◄── /scan  │
│  Wall Fit   │                                                   │
│             │    /cmd_vel (DOCKING phase - direct control)      │
│             │ ─────────────────────────────────────────────────►│
└─────────────┘
```

**State Machine**: `IDLE → DOCKING → DOCKED / FAILED`

- **IDLE**: Publishes Nav2 goal, monitors distance + orientation to target. Transitions to DOCKING when within 0.3m and orientation error < ~20°.
- **DOCKING**: Takes direct control via `/cmd_vel`. Uses RANSAC line fitting on front-facing LiDAR points (±60°) to extract wall geometry. Two-phase control:
  - *Approach*: Simultaneous linear + angular correction
  - *Final* (< 0.2m): Sequential — rotate first, then inch forward
- **DOCKED**: Confirmed when distance and angle errors are within tolerance for 5 consecutive cycles with > 80% RANSAC confidence.
- **FAILED**: Triggered by 25 consecutive wall detection failures or 30s timeout.

## Prerequisites

```bash
# ROS2 Humble + TurtleBot3 + Nav2
sudo apt install ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup

# TurtleBot3 packages
sudo apt install ros-humble-turtlebot3* ros-humble-turtlebot3-msgs
export TURTLEBOT3_MODEL=burger
```

## Build

```bash
cd ~/<your_ros2_ws>/src
git clone https://github.com/chngyicheng/docking-demo.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/<your_ros2_ws>
colcon build --symlink-install
source install/setup.bash
```

## Running the Demo

### Step 1: Launch Simulation + Navigation

> **Note**: Update the path to your workspace in the scripts before launching.

```bash
./docking_demo/scripts/startup_turtlebot.sh
```

This opens a tmux session with 4 panes: Gazebo, Nav2, Cartographer, and teleop. Use teleop to drive around and build a map, then save it.

> **Note**: The startup script expects a pre-saved map at `~/map.yaml`. Generate one first with Cartographer + `ros2 run nav2_map_server map_saver_cli`.

### Step 2: Set Initial Pose

Set the robot's initial pose estimate in RViz2 so Nav2 can localize.

### Step 3A: Normal Demo (no disturbances)

```bash
./docking_demo/scripts/run_normal_demo.sh
```

This launches both the docking node and the disturbance node (as a passthrough with all disturbances disabled).

### Step 3B: Demo with Disturbances

```bash
./docking_demo/scripts/run_demo_w_disturbance.sh
```

This launches the same nodes but with scan dropout and corruption enabled.

## Docking Target

The default target is configured in `config/params.yaml`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `target_x` | 1.7 | Map x-coordinate |
| `target_y` | -1.3 | Map y-coordinate |
| `target_theta` | -1.6 rad (~-92°) | Desired heading at dock |
| `approach_distance_threshold` | 0.3m | Distance to switch from Nav2 to LiDAR control |
| `final_approach_distance` | 0.2m | Distance to enter final phase (sequential control) |
| `docking_distance_threshold` | 0.05m | Success criterion — position |
| `docking_angle_threshold` | 0.05 rad (~3°) | Success criterion — orientation |

## Disturbances

The `DisturbanceNode` sits between `/scan` and `/scan_disturbed`, applying time-windowed disturbances:

| Disturbance | Behavior | Timing |
|-------------|----------|--------|
| **Scan Dropout** | Drops all scans during burst windows | 0.6s bursts every 1.5–4.0s (randomized) |
| **Scan Corruption** | Sets front ±50° sector to `inf` | 1.0s bursts every 2.0–5.0s (randomized) |

Safety behaviors under disturbances:
- **Stale scan** (> 0.5s old): Robot stops immediately
- **Low RANSAC confidence** (< 60%): Robot stops and waits
- **Consecutive failures** (≥ 25): Transitions to FAILED state

## Control Parameters (Hardcoded)

| Parameter | Value |
|-----------|-------|
| Control loop rate | 50 Hz |
| Linear gain (`k_linear`) | 0.3 |
| Angular gain (`k_angular`) | 1.5 |
| Target wall distance (`d_target`) | 0.1m |
| Max linear velocity | 0.15 m/s |
| Max angular velocity | 0.5 rad/s |
| Distance tolerance | 0.05m |
| Angle tolerance | 0.09 rad (~5°) |
| RANSAC iterations | 50 |
| RANSAC inlier threshold | 0.05m |
| Min inlier ratio | 30% |

## Package Structure

```
docking_demo/
├── include/docking_demo/
│   ├── docking_node.hpp        # Docking state machine + RANSAC
│   └── disturbance_node.hpp    # Scan disturbance injector
├── src/
│   ├── docking_main.cpp
│   ├── docking_node.cpp
│   ├── disturbance_main.cpp
│   └── disturbance_node.cpp
├── launch/
│   └── docking_demo.launch.py
├── config/
│   └── params.yaml
├── scripts/
│   ├── startup_turtlebot.sh    # Gazebo + Nav2 + Cartographer
│   ├── run_normal_demo.sh
│   └── run_demo_w_disturbance.sh
├── CMakeLists.txt
└── package.xml
```

## Known Limitations

- **Single wall assumption**: RANSAC fits one dominant line. In multi-wall environments, it may lock onto an unintended wall if orientation gating is too loose.
- **No retry logic**: If docking enters FAILED, the node must be restarted. There is no automatic recovery or re-approach.
- **Hardcoded control gains**: Gains are not exposed as ROS parameters — tuning requires recompilation.
- **No lateral alignment**: The controller aligns perpendicular distance and heading to the wall, but does not correct lateral offset along the wall.
- **LiDAR minimum range**: TurtleBot3 LDS-01 has ~12cm minimum range, limiting how close the robot can reliably sense the wall. `d_target` is set to 0.1m to stay above this.

## Troubleshooting

**Robot doesn't move**: Ensure `TURTLEBOT3_MODEL=burger` is exported and Nav2 has a valid initial pose estimate.

**Docking node receives no scans**: Verify the disturbance node is running — it bridges `/scan` to `/scan_disturbed`. Check with `ros2 topic echo /scan_disturbed`.

**RANSAC confidence always low**: Check that the robot is facing a wall within the ±60° front scan window. Open areas or corners at oblique angles produce poor line fits.

**Build fails**: Ensure `source /opt/ros/humble/setup.bash` and that `eigen3_cmake_module` is installed.
