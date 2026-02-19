# Path Planning & Control Integration Guide

## Quick Start

### 1. Launch the Simulation
```bash
# Terminal 1: Start the EUFS simulator
cd ~/uh-fs-ai
. install/setup.bash
ros2 launch eufs_launcher simulation.launch.py
```

The simulator will:
- Start publishing `/cones` topic with blue/yellow cone positions
- Publish `/ground_truth/state` with car odometry
- Wait for `/cmd` commands from control

### 2. Run the Planning Node
```bash
# Terminal 2: Start the planner
cd ~/uh-fs-ai
. install/setup.bash
ros2 run planning planning_node
```

Expected output:
```
[planning_node]: Starting local_planner...
[planning_node]: Processing cone data...
```

### 3. Run the Control Node
```bash
# Terminal 3: Start the controller
cd ~/uh-fs-ai
. install/setup.bash
ros2 run control control_node
```

The control system will:
- Read waypoints from `/trajectory` (published by planning_node)
- Read car state from `/ground_truth/state`
- Calculate steering angle using pure pursuit
- Calculate throttle using speed PID
- Publish commands to `/cmd` for the simulator

## Data Flow

```
Simulator
   ↓
[/cones topic] → Planning Node
                    ↓
              [Extract cones]
              [Find centerline]
              [Interpolate path]
                    ↓
              [/trajectory topic] → Control Node
                                       ↓
                                  [Pure pursuit]
                                  [Speed control]
                                       ↓
                                  [/cmd topic] → Simulator
                                  
[/ground_truth/state] → Control Node
```

## Monitoring in RViz

```bash
# Terminal 4: Open visualization
ros2 run rviz2 rviz2 -d <path>/rviz_config.rviz
```

Set frame to `base_footprint` and subscribe to:
- `/trajectory` → Waypoint array (white dots)
- `/planner/viz` → Midpoints (green dots)
- `/planner/LineStrip` → Interpolated path (cyan line)
- `/planner/LineList` → Blue/Yellow connections (red lines)
- `/control/viz` → Current steering angle (marker)

## Testing Parameters

### For Tight Turns
```bash
ros2 param set /local_planner track_width 5.0
ros2 param set /local_planner cone_spacing 3.0
```

### For Straights
Same parameters work well.

### Adjust Control Response
```bash
# Increase lookahead for smoother curves
ros2 param set /control look_ahead 5.0

# Increase steering gain (carefully!)
ros2 param set /control K_p 1.2

# Adjust max speed
ros2 param set /control max_speed 4.5
```

## Debugging Issues

### No Waypoints Being Published
```bash
# Check if planning node is receiving cones
ros2 topic echo /cones --field blue_cones | head -5
```

- If empty: Simulation isn't publishing cones (check launcher)
- If present: Check planning node logs for interpolation errors

### Car Not Following Trajectory
1. Check `/trajectory` has 50+ waypoints
2. Verify `/cmd` is being published
3. Monitor `/control/Index` marker - should move along path

### Jerky Control Output
- Increase interpolation smoothing (reduce `s=50` parameter)
- Decrease control gains (`K_p`, `K_d`)
- Increase lookahead distance

## Message Formats

### Input: `/cones` (ConeArrayWithCovariance)
```
blue_cones: [ConeWithCovariance]      # Left boundary
yellow_cones: [ConeWithCovariance]    # Right boundary
orange_cones: [ConeWithCovariance]    # Start/end markers
big_orange_cones: [ConeWithCovariance]
unknown_color_cones: [ConeWithCovariance]

Each cone contains:
  point: {x, y, z}  # Position in car frame
  covariance: 3x3   # Uncertainty matrix
```

### Output: `/trajectory` (WaypointArrayStamped)
```
header:
  frame_id: "base_footprint"
  stamp: timestamp
waypoints: [Waypoint]
  
Each waypoint contains:
  position: {x, y}  # Target position
```

### Control Output: `/cmd` (AckermannDriveStamped)
```
header:
  frame_id: "base_footprint"
  stamp: timestamp
drive:
  steering_angle: float    # Radians (-30° to +30°)
  steering_angle_velocity: float
  speed: float             # m/s (0 to 4.5)
  acceleration: float      # m/s² (for deceleration)
  jerk: float             # Rate of acceleration change
```

## Performance Targets

- **Waypoint Update Rate**: 20 Hz (matches `/cones` at 50ms)
- **Path Latency**: < 100ms from cone detection to control
- **Waypoint Density**: 50-500 points/lap
- **Lap Completion**: < 30 seconds for typical track

## Troubleshooting Checklist

- [ ] Simulator launching successfully
- [ ] `/cones` topic has data (use `ros2 topic hz /cones`)
- [ ] `/trajectory` has 50+ waypoints
- [ ] `/cmd` topic shows drive commands
- [ ] Car moves in RViz
- [ ] Car follows waypoints visually
- [ ] Smooth steering commands (no jerkiness)

## Log Analysis

```bash
# Monitor planning latency
ros2 topic hz /trajectory

# Check waypoint count per update
ros2 topic echo /trajectory --field waypoints. --repeated-field-expansion

# Monitor control commands
ros2 topic echo /cmd --field drive.steering_angle
```

## Contact & Support

For issues with:
- **Path Planning**: Check [ALGORITHM.md](./ALGORITHM.md)
- **Control Logic**: See control_node.py comments
- **Simulation**: Refer to eufs_launcher documentation
