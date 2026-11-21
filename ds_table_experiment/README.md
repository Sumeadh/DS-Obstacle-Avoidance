# UR5e dynamical-system obstacle-avoidance experiments

This standalone ROS 2 package uses its own Gazebo world and the standard UR5e
MoveIt configuration. It does not launch or modify the
`sorting_description` application.

The controller combines the nominal Cartesian dynamical system
`xdot = gain * (goal - x)` with ellipsoid obstacle modulation. Gazebo
initializes the robot at A before activating the joint-state broadcaster and
velocity controller. Cartesian velocity commands are streamed through MoveIt
Servo and `forward_velocity_controller`.

The Gazebo table and red barrier are visual-only in the supplied world
(the barrier collision element is commented out). The DS controller performs
the obstacle avoidance mathematically using the configured ellipsoid. The
controller's `add_obstacle()` helper is retained, but is not called in the
normal experiment launch; MoveIt Servo's scene collision checker therefore
does not provide an independent barrier collision constraint in this mode.

## Run

```bash
cd ~/Projects/ur5e_sorting_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ds_table_experiment
source install/setup.bash
ros2 launch ds_table_experiment nominal_ab_clean.launch.py
```

`setup_mode` defaults to `false`: it activates
`forward_velocity_controller` and starts the velocity experiment using the
joint pose and controller settings from `config/controller_params.yaml`.
Set `setup_mode:=true` to activate
`joint_trajectory_controller` for RViz setup; this adds the red obstacle to
MoveIt's planning scene without starting the automatic velocity experiment.
`nominal_ab.launch.py` remains an alias to this launch for compatibility.

The table is 0.70 m by 2.00 m with a 0.10 m-thick top. Its center is at
`(0.65, 0.0, 0.45) m`, so the tabletop is at `z=0.50 m`. Gazebo also shows
visual-only orange A and green B spheres at the configured start and target
locations. Update their poses in `worlds/table.world` if the experiment
geometry changes.

Tune the controller in `config/controller_params.yaml`: `gain`, `speed_limit`,
`control_period`, `position_tolerance`, `maximum_steps`, the MoveIt obstacle
dimensions and position, and `goal_offset` (relative X/Y/Z meters from the
measured tool start). The controller sends linear velocity commands in X, Y,
and Z; angular velocity remains zero. The Gazebo obstacle and table remain defined in
`worlds/table.world`; keep the obstacle values synchronized if you change them.
The `initial_positions` section in the same config sets the six robot joint
angles in radians. The launch file supplies those values to the URDF's
initial-position input and the controller/scene nodes.
Use `gui:=false` to disable Gazebo's GUI or `rviz:=false` to disable RViz.

The nominal velocity field runs at 50 Hz with a 0.04 m/s speed cap and a 5 mm
goal tolerance. Servo's collision checking uses the red obstacle in MoveIt's
planning scene; the tabletop is not a collision object.

## Code layout

- `scripts/algorithms/nominal_ds.py`: pure math for the nominal velocity field.
- `scripts/Controller.py`: obstacle scene and velocity task flow.
- `config/servo_velocity.yaml`: Servo's joint-velocity output configuration.
- `config/controller_params.yaml`: experiment tuning and initial joint pose.

The math can be unit tested without ROS or a robot:

```bash
cd ~/Projects/ur5e_sorting_ws/src/ds_table_experiment
PYTHONPATH=scripts python3 -m unittest discover -s scripts/tests
```

## Experiment 1: adaptation to a changing target

The controller starts toward the configured target B. While it is moving,
publish a `geometry_msgs/PointStamped` message to `/new_target` in
`base_link`. The controller updates its goal immediately and continues the
same servo loop; when it reaches a target it holds position with zero velocity
until another target arrives.

Example: after the robot has started moving, send target C two seconds later:

```bash
ros2 run ds_table_experiment target_publisher.py --ros-args \\
  -p target_position:="[0.523, -0.363, 0.543]" \\
  -p delay_seconds:=2.0
```

The publisher defaults to a 12 cm positive-X shift from B. Change
`target_position` to test other target shifts. Start this publisher only after
the controller has begun moving toward B; otherwise the initial startup may
overwrite an early target with B.

On shutdown, the controller reports the final tool-to-current-goal error and
the minimum signed clearance of the tool point from the physical box. A
negative clearance means the tool point entered the box; this is a point-to-box
metric, not a full robot-body collision clearance.


### Visualize the changed target in RViz

The controller publishes the latest received target as a green 4 cm sphere on
`/experiment/target_marker` (`visualization_msgs/Marker`). In RViz, set the
Fixed Frame to the controller's `planning_frame` (normally `base_link`), add a
**Marker** display, and select `/experiment/target_marker`. When `/new_target`
is received, the sphere moves to the new target position.
