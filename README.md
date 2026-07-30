# Robot Bringup — Real-Robot Navigation

Minimal FineNav-Engine navigation stack for live robot testing.  Built around
real hardware components instead of simulation stubs.

```
/odom → FineNavLocalizer (ESKF) → BtNavigator → ComputePlan("mppi_layer")
                                                    ↓
                                     route_layer: AstarPathSearch (SingleShot)
                                                    ↓
                                     mppi_layer:  MPPIController (50 Hz Tracking)
                                                    ↓
                                               /cmd_vel
/goal_pose → FineNavEngine → navigateTo()
/map       → OccupancyGrid2D  → AstarMapView / FusedMppiMapView
/point_cloud → TemporalVoxelMap → FusedMppiMapView (dynamic obstacles)
```

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- FineNav-Engine workspace built: `colcon build --symlink-install && source install/setup.bash`
- **A real robot** with:
  - Odometry source publishing to a topic (e.g. `/Odometry` from LiDAR/IMU SLAM)
  - An occupancy grid map on `/map` (from SLAM or `map_server`)
  - A `/tf` tree with `base_link` → sensor frames
- **Optional but recommended:** Rviz2 for visualisation and goal-pose publishing

## Quick Start

### 0. Build

```bash
cd ~/FineNav_Engine_ws
colcon build --symlink-install --packages-select finenav_robot_bringup
source install/setup.bash
```

### 1. Ensure a map is being published

You need an `OccupancyGrid` on `/map`.  Either run SLAM (e.g. FastLIO + pointlio,
slam_toolbox) **or** serve a pre-built map:

```bash
# Option A: pre-built map
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/path/to/map.yaml

# Option B: SLAM is already running and publishes /map
```

### 2. Ensure odometry is available

The default config listens on `/Odometry` with mode `relative_pose`.
Edit `config/robot_params.yaml` if your robot uses a different topic or mode.

### 3. Launch the bringup

```bash
ros2 launch finenav_robot_bringup robot_bringup.launch.py
```

Or run the node directly with a custom params file:

```bash
ros2 run finenav_robot_bringup robot_bringup \
  --ros-args --params-file src/finenav_robot_bringup/config/robot_params.yaml
```

### 4. Set initial pose

Use Rviz2's "2D Pose Estimate" tool to set the robot's initial pose on the map.
This publishes to `/initialpose`.

```bash
# Or manually:
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: { frame_id: 'map' },
  pose: {
    pose: {
      position: { x: 0.0, y: 0.0, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]
  }
}"
```

### 5. Send a navigation goal

Use Rviz2's "2D Goal Pose" tool (publishes to `/goal_pose`), or publish manually:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: { frame_id: 'map' },
  pose: {
    position: { x: 5.0, y: 3.0, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  }
}"
```

The robot should plan a route via A* and track it with MPPI.  Verify with:

```bash
ros2 topic echo /cmd_vel
```

## How It Works

`FineNavEngine` subscribes to `/goal_pose` and triggers `navigateTo()`.  The BT
ID is set in `main.cpp`:

```cpp
engine->setDefaultBehaviorTree("RobotNavigateToPose");
```

The engine assembles a two-layer planning pipeline:

1. **Route Layer** (`route_layer`, `AstarPathSearch` with `SingleShotPolicy`):
   plans a collision-free path from the current pose to the goal using A* on
   the occupancy grid.

2. **Tracking Layer** (`mppi_layer`, `MPPIController` with `ContinuousPolicy{50.0}`):
   tracks the reference path at 50 Hz, outputting `/cmd_vel` via the
   navigator's chassis-command interface.

The behaviour tree (`behavior_trees/navigate_to_pose.xml`) references only the
final tracking layer — the framework automatically chains the route planner
upstream when the pipeline is wired:

```cpp
engine->wireControlPipeline({
    _root >> route_layer >> mppi_layer
});
```

### Map Adapters

Three lightweight adapter classes bridge `OccupancyGrid2D` and `TemporalVoxelMap` to each planner's
`IMapView`:

| Adapter | Implements | Used by |
|---|---|---|
| `AstarMapView` | `astar::IMapView` | `AstarPathSearch` |
| `MPPIMapView`  | `nav2_mppi_controller::IMapView` | `MPPIController` (static only) |
| `FusedMppiMapView` | `nav2_mppi_controller::IMapView` | `MPPIController` (static + dynamic) |

`FusedMppiMapView` is used by default — it fuses the static `OccupancyGrid2D`
with the dynamic `TemporalVoxelMap` (LiDAR point cloud), returning the most
conservative cost. All MapViews use a **circular footprint model** (default
radius 0.3 m). Adjust `robot_radius_` if your robot is larger.

## Architecture Comparison — Simulation vs Real Robot

| Aspect | Getting Started (Sim) | Robot Bringup (Real) |
|---|---|---|
| Map | `DummyMap` (never occupied) | `OccupancyGrid2D` (live `/map`) + `TemporalVoxelMap` (LiDAR) |
| Planner | `DummyPlanner` (twist-to-goal) | `AstarPathSearch` + `MPPIController` |
| Odometry | Sim ground truth `/odom` | Real LiDAR/IMU SLAM `/Odometry` |
| TF | `publish_tf: false` (sim provides) | `publish_tf: true` (localizer broadcasts) |
| Time | `use_sim_time: true` | `use_sim_time: false` |
| Footprint | None | Circular (configurable radius) |
| Pipeline | 1 layer | 2 layers (route → track) |

## Key Parameters (`robot_params.yaml`)

| Section | Parameter | Purpose |
|---|---|---|
| `robot_bringup/` | `use_sim_time: false` | Use wall clock |
| `finenav_localizer` | `odom0: "/Odometry"` | Your robot's odometry topic |
| | `publish_tf: true` | Broadcast `map→base_link` |
| | `force_2d_mode: true` | Planar constraint |
| | `model.pos_noise_std: 0.1` | Higher noise for real sensors |
| `astar_planner` | `connectivity: "8"` | 8-connected grid search |
| | `heuristic: "Euclidean"` | Distance heuristic |
| `finenav_mppi_controller` | `motion_model: "DiffDrive"` | Differential-drive kinematics |
| | `vx_max: 0.5` | Max forward speed [m/s] |
| | `wz_max: 1.9` | Max angular speed [rad/s] |
| `occupancy_grid_2d` | `occupancy_threshold: 65` | Cells ≥ this value are obstacles |
| `finenav_navigator` | `behavior_trees` | Load the bringup's BT XML |

### Tuning Notes

- **Process noise**: Start with the defaults above.  If the ESKF drifts, increase
  `pos_noise_std` / `rot_noise_std` to trust the model more.
- **Speed limits**: Set `vx_max`, `wz_max` to match your robot's physical limits.
- **MPPI critics**: Disable critics you don't need (e.g. `ObstaclesCritic` if
  using `CostCritic` with inflation).  Adjust `cost_weight` for each critic to
  balance goal-seeking vs obstacle avoidance.
- **Footprint radius**: Edit `setRobotRadius()` in `mppi_map_view.cpp` or add a
  ROS parameter in `MPPIMapView` if your robot is not circular.

## Troubleshooting

| Symptom | Check |
|---|---|
| Robot doesn't move | `ros2 topic echo /cmd_vel` — is it publishing non-zero? |
| | `ros2 topic echo /goal_pose` — is the goal being received? |
| | Ensure a map is published on `/map` |
| No TF | Ensure `publish_tf: true` in params |
| | Check that your odometry source publishes to the configured topic |
| "behavior tree ID is empty" | `setDefaultBehaviorTree()` was not called |
| "Failed to create tree" | Ensure `behavior_trees` includes the bringup's BT directory |
| Localizer drifts | Increase `pos_noise_std` / `rot_noise_std` |
| | Verify `odom0_mode` matches your sensor (pose vs relative_pose) |
| MPPI doesn't follow path | Check `motion_model` matches your robot |
| | Increase `PathAlignCritic.cost_weight` or `PathFollowCritic.cost_weight` |
| Robot oscillates near goal | Decrease `GoalCritic.threshold_to_consider` |
| | Increase `yaw_tolerance` in BT |
