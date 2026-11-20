# Nominal UR5e A-to-B table experiment

This standalone ROS 2 package uses its own Gazebo world and the standard UR5e
MoveIt configuration. It does not launch or modify the
`sorting_description` application.

The nominal Cartesian dynamical system for the A-to-B segment is
`xdot = gain * (B - x)`, with a configurable speed cap. There is no obstacle
modulation yet. Gazebo initializes the robot directly at A before activating
the joint-state broadcaster and velocity controller in experiment mode. The
controller adds the obstacle to MoveIt's planning scene, then streams Cartesian
velocity commands through MoveIt Servo and `forward_velocity_controller`. No
move-to-A trajectory or controller switch is used.

The Gazebo table is visual-only and has no collision geometry. The red
0.70 × 0.10 × 0.20 m barrier is the only environment collision object in both
Gazebo and MoveIt's planning scene. The unmodulated nominal controller does
not avoid it and may press the simulated robot against it; use this setup only
for supervised simulation testing until obstacle modulation is implemented.

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
`(0.65, 0.0, 0.45) m`, so the tabletop is at `z=0.50 m`.
Gazebo also shows visual-only orange A and green B spheres at the current
measured start pose and the controller's 0.50 m negative-X goal. Update their
poses in `worlds/table.world` if the initial arm pose changes.

Tune the controller in `config/controller_params.yaml`: `gain`, `speed_limit`,
`control_period`, `position_tolerance`, `maximum_steps`, the MoveIt obstacle
dimensions and position, and `goal_offset` (relative X/Y meters from the
measured tool start). The Gazebo obstacle and table remain defined in
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
