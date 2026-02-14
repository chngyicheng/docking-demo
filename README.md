# Robust Docking with LiDAR-Based Pose Refinement

Precision docking system using LiDAR geometry extraction for final approach (< 1m), tested under pose noise and scan disturbances.

## Prerequisites
```bash
# ROS2 Humble + TurtleBot3
sudo apt install ros-humble-desktop ros-humble-turtlebot3*

# Set model
export TURTLEBOT3_MODEL=burger
```

## Build & Run
```bash
git clone https://github.com/chngyicheng/docking-demo.git
cd ~/docking-demo
colcon build
source install/setup.bash

# One-command demo
./docking_demo/scripts/run_demo.sh
```

## Expected Behavior

1. **Approach phase**: Robot navigates to ~1.5m from target using odometry
2. **Refinement phase**: Switches to LiDAR-based control, extracts wall/corner geometry
3. **Final dock**: Converges to target pose within 5cm position, 3° orientation
4. **With disturbances**: May retry 1-2 times but still achieves final precision

## Testing Disturbances
```bash
# Enable specific disturbances
ros2 launch docking_demo docking_demo.launch.py \
  pose_noise:=true \
  scan_dropout:=true
```

Available disturbances:
- `pose_noise` - Adds ±10cm noise to odometry-based pose estimate
- `scan_dropout` - Randomly drops 30% of laser scans
- `scan_corruption` - Sets random sectors to infinity

## Docking Target

Corner at `(1.5, 2.0)` in turtlebot3_world map (inner wall corner near center).

## Documentation

- `docs/design_note.pdf` - Architecture and control law explanation
- `docs/demo_recording.mp4` - Screen recording of baseline + disturbance tests

## Troubleshooting

**Robot doesn't move**: Check `TURTLEBOT3_MODEL` is set to `burger`

**Build fails**: Ensure `source /opt/ros/humble/setup.bash` before building
