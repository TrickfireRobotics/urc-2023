# NAV2 Parameters Guide

## What is a ROS2 Parameters File?

A ROS2 parameters file (like `nav2_params.yaml`) defines configuration values for ROS2 nodes at startup. It uses YAML format to structure hierarchical data. When a node launches, it reads these parameters and uses them to configure its behavior.

## YAML Structure

In YAML, indentation matters. Each level of indentation represents a nested hierarchy:
```
parent_node:
  ros__parameters:
    parameter_name: value
    nested_group:
      sub_parameter: value
```

The `ros__parameters:` key is required by ROS2 and indicates where node parameters begin.

---

## File Structure Overview

This `nav2_params.yaml` file configures the Navigation2 stack, which handles autonomous navigation for your robot. It has **4 main sections**:

```
controller_server       → Local path following (receives global path, outputs velocity commands)
planner_server          → Global path planning (finds path from start to goal)
local_costmap           → Local obstacle map (around the robot, for collision avoidance)
global_costmap          → Global obstacle map (entire environment, for planning)
```

---

## Section 1: Controller Server (Local Path Planner)

**What it does:** Takes the global path from the planner and converts it into velocity commands that keep the robot following that path while avoiding obstacles.

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0        # How often to compute velocity commands (Hz)
    
    # These define how close the robot needs to be to considered "arrived"
    min_x_velocity_threshold: 0.001   # Minimum forward velocity to be considered moving
    min_y_velocity_threshold: 0.5     # Minimum sideways velocity (0 = can't move sideways)
    min_theta_velocity_threshold: 0.001 # Minimum rotation velocity
    
    # Plugins that check progress and goal achievement
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]  # The actual path-following algorithm
    
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5    # Robot must move this far (meters) or get stuck
      movement_time_allowance: 10.0    # Time allowed (seconds) to achieve that movement
    
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25          # Distance (meters) to be within goal
      yaw_goal_tolerance: 0.25         # Rotation (radians) to be within goal
      stateful: True                   # Remember previous state
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"  # DWB = Dynamic Window Approach
      
      # Velocity limits (m/s for linear, rad/s for angular)
      min_vel_x: 0.0                   # Can go forward or stop
      max_vel_x: 0.08                  # Max forward speed
      min_vel_y: 0.0                   # No sideways movement
      max_vel_y: 0.0
      max_vel_theta: 1.0               # Max rotation speed
      
      # Speed constraints (more restrictive than above)
      max_speed_xy: 0.07               # Practical max speed
      min_speed_xy: 0.0
      
      # Acceleration/deceleration limits (m/s²)
      acc_lim_x: 1.0                   # How quickly to speed up forward
      decel_lim_x: -1.0                # How quickly to slow down
      acc_lim_theta: 3.2               # How quickly to rotate
      decel_lim_theta: -3.2
      
      # DWB algorithm samples these many velocities to find the best path
      vx_samples: 20                   # Forward velocities to evaluate
      vy_samples: 5                    # Sideways velocities (all 0 anyway)
      vtheta_samples: 20               # Rotation velocities to evaluate
      
      sim_time: 1.7                    # Simulate trajectory this far into future (seconds)
      linear_granularity: 0.05         # Resolution of forward speed sampling
      angular_granularity: 0.025       # Resolution of rotation sampling
      
      # Goal tolerance
      xy_goal_tolerance: 0.25          # Distance to goal before stopping
      
      # Collision checking
      trans_stopped_velocity: 0.25     # Velocity threshold: consider robot stopped
      short_circuit_trajectory_evaluation: True  # Stop evaluating if collision found
      limit_vel_cmd_in_traj: False     # Don't limit velocity within trajectory
      
      # Critics are scoring functions that evaluate quality of trajectories
      critics:
        - "RotateToGoal"               # Prefer rotating to face goal
        - "Oscillation"                # Penalize oscillating/zigzagging
        - "BaseObstacle"               # Penalize getting near obstacles
        - "GoalAlign"                  # Prefer heading toward goal
        - "PathAlign"                  # Prefer aligned with planned path
        - "PathDist"                   # Prefer staying close to path
        - "GoalDist"                   # Prefer moving toward goal
      
      # Weight/scale for each critic (higher = more important)
      BaseObstacle.scale: 0.2
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0 # Rotate slower when approaching goal
```

---

## Section 2: Planner Server (Global Path Planner)

**What it does:** Plans a path from the robot's current position to a goal location, accounting for the global map.

```yaml
planner_server:
  ros__parameters:
    planner_frequency: 1.0            # How often to replan (Hz) - 1 = once per second
    expected_planner_frequency: 1.0   # Expected frequency (for diagnostics)
    
    planner_plugins: ["GridBased"]    # Name of the planner to use
    
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"  # A* algorithm on grid
      tolerance: 0.25                 # How close to goal in meters
      use_astar: true                 # Use A* (faster) instead of Dijkstra
      allow_unknown: true             # Can plan through unexplored areas
      use_final_approach_orientation: true  # Orient robot toward goal at end
```

---

## Section 3: Local Costmap

**What it does:** Creates a small map (3m x 3m around the robot) that tracks nearby obstacles for quick collision avoidance.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0           # How often to update map (Hz)
      publish_frequency: 2.0          # How often to publish for visualization
      
      global_frame: odom              # Reference frame (odometry/world coordinates)
      robot_base_frame: zed_camera_link  # Robot's center/reference point
      transform_tolerance: 0.1        # Tolerance for frame transformations (seconds)
      
      rolling_window: true            # Window moves with robot (not fixed to map)
      width: 3                        # Map size in meters
      height: 3
      resolution: 0.05                # Each grid cell is 0.05m x 0.05m
      
      # Layers that build the costmap
      plugins: ["obstacle_layer", "inflation_layer"]
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: filtered_cloud  # Input data source name
        filtered_cloud:
          topic: /filtered_point_cloud  # Where to listen for point cloud
          sensor_frame: zed_left_camera_frame  # Frame of the sensor
          data_type: "PointCloud2"    # ROS2 data type
          marking: true               # Add obstacles where points detected
          clearing: true              # Remove obstacles where no points
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55        # Expand obstacles by this distance
        cost_scaling_factor: 1.0      # How aggressively to penalize near obstacles
```

---

## Section 4: Global Costmap

**What it does:** Creates a full map of the environment (from SLAM or pre-loaded map) to plan paths that avoid obstacles everywhere, not just nearby.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True  # Subscribe to map updates
        enabled: true
        subscribe_to_updates: true   # Update as map changes
        transform_tolerance: 0.1
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: filtered_cloud
        filtered_cloud:
          topic: /filtered_point_cloud
          sensor_frame: zed_left_camera_frame
          data_type: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.1    # Only mark obstacles above 10cm
          max_obstacle_height: 2.0    # Only mark obstacles below 2m
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55
        cost_scaling_factor: 1.0
        inflate_unknown: false        # Unknown areas aren't inflated
        inflate_around_unknown: true  # But inflate around unknown areas
        always_send_full_costmap: True  # Always send entire map
        introspection_mode: "disabled"  # Disable internal diagnostics
```

---

## How They Work Together

```
1. Planner Server (Global Planner)
   ↓ Uses global costmap to find path
   ↓ Outputs waypoints/path to goal
   ↓
2. Controller Server (Local Planner)
   ↓ Receives path from planner
   ↓ Uses local costmap to avoid nearby obstacles
   ↓ Outputs velocity commands
   ↓
3. Robot
   ↓ Follows velocity commands
   ↓ Sends sensor data (point cloud)
   ↓
4. Both Costmaps
   ↓ Update using sensor data and existing map
   ↓ Feed back to planners
```

---

## Common Parameters You Might Tune

| Parameter | Purpose | Effect of Increasing |
|-----------|---------|----------------------|
| `max_vel_x` | Max forward speed | Robot moves faster |
| `acc_lim_x` | Forward acceleration | Robot speeds up/slows down quicker |
| `inflation_radius` | Safety buffer around obstacles | Robot stays further from obstacles |
| `xy_goal_tolerance` | How close to stop at goal | Closer = more precise but may oscillate |
| `vx_samples`, `vtheta_samples` | Velocity sampling for DWB | More samples = better quality but slower |
| `sim_time` | How far to look ahead | Longer = smoother but more compute |

