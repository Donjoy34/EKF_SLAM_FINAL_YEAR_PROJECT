# Path Planning Algorithm Documentation

## Overview
The planning node implements a **midpoint-based trajectory planning algorithm** that processes cone positions from the EUFS simulation and generates smooth waypoints for the control system.

## Algorithm Steps

### 1. **Cone Classification & Preprocessing**
- **Input**: Cone positions from `/cones` topic (blue, yellow, orange, uncolored)
- **Process**:
  - Blue cones → Left track boundary
  - Yellow cones → Right track boundary
  - Uncolored/orange cones classified by position (imag > 0 → left, < 0 → right)
  - Distance filtering to keep only nearby cones (< 4-10m)

### 2. **Centerline Computation**
Creates midpoints between left and right boundaries:
```
For each blue-yellow cone pair:
  midpoint = (blue_cone + yellow_cone) / 2
```

**Robust handling of missing cones**:
- If no yellow cones: offset blue cones by track_width/2 to the right
- If no blue cones: offset yellow cones by track_width/2 to the left
- If unequal counts: handle excess cones with appropriate offsets

### 3. **Path Sorting**
Sort midpoints by angular direction (atan2) from vehicle position:
- More robust than distance-only sorting
- Handles track loops and complex geometries better
- Ensures smooth progression along the track

### 4. **Trajectory Interpolation**
**For 4+ waypoints** (Cubic Spline):
```python
splprep(points, k=3, s=50)  # k=3 for smooth curves
waypoints = 5× original count  # Increase resolution
```

**For 2-3 waypoints** (Linear Interpolation):
```python
20 waypoints distributed along path segments
```

**Benefits**:
- Smoother control inputs (less aggressive steering)
- Better handling of curves
- Higher control frequency resolution

### 5. **Output Publishing**
**Topic**: `/trajectory` (Type: `WaypointArrayStamped`)
- Published at callback rate (20Hz from `/cones`)
- Contains X, Y positions in `base_footprint` frame
- Ready for control node consumption

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `track_width` | 5.0 m | Distance between left and right boundaries |
| `cone_spacing` | 3.0 m | Expected gap between consecutive cones |
| `lookahead_distance` | 10.0 m | Forward planning horizon |
| `threshold` | 6.0 m | Max cone detection distance |

## Visualization Markers

- **Green Points**: Original midpoints (base trajectory)
- **Cyan Line**: Interpolated smooth path (final trajectory)
- **Red Lines**: Cone-to-pole connections (for debugging)

View in RViz under `base_footprint` frame.

## Failure Modes & Recovery

| Scenario | Handling |
|----------|----------|
| No cones detected | Return early (no waypoint update) |
| Only one boundary visible | Offset from available side |
| Interpolation fails | Use raw midpoints |
| Empty waypoint array | Skip publish |

## Performance Characteristics

- **Latency**: ~50-100ms (single callback)
- **Waypoint density**: 50-500 waypoints depending on track visibility
- **Update rate**: Tied to `/cones` publish rate (typically 20Hz)
- **Memory**: O(n) where n = number of visible cones

## Integration with Control Node

The control node subscribes to `/trajectory` topic and:
1. Extracts waypoint positions
2. Computes desired steering angle (pure pursuit or similar)
3. Computes desired velocity (curvature-based)
4. Publishes control commands

### Expected Control Behavior
- **Smooth turns**: Handles curves with ~5m+ radius
- **Lane keeping**: Centers vehicle between boundaries
- **Reactive**: Responds to track changes within 2-3 steps

## Testing & Tuning

### Verify Output
```bash
# Monitor waypoints in RViz
ros2 topic echo /trajectory --field waypoints[0]
```

### Adjust Smoothing
- Increase `s` parameter in splprep for less aggressive interpolation
- Decrease for tighter curves
- Default `s=50` works for typical 3-meter cone spacing

### Debug Visualization
```bash
# View all markers
ros2 viz --frame base_footprint
```

## Future Enhancements

1. **Velocity Planning**: Add speed suggestions based on path curvature
2. **Adaptive Lookahead**: Dynamic window based on vehicle state
3. **Multiple Path Candidates**: Generate alternative trajectories for lane choice
4. **Machine Learning**: Learn optimal centerline from driver data
5. **Cone Confidence**: Weight waypoints by cone detection certainty

## References

- **Lines 29-100**: Main callback and algorithm orchestration
- **Lines 120-172**: Centerline computation logic
- **Lines 221-280**: Interpolation engine
- **Lines 300+**: Publishing to ROS topics
