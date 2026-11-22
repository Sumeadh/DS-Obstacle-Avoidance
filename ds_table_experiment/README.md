# UR5e Dynamical System Obstacle Avoidance

This ROS 2 package implements a Cartesian dynamical system (DS) controller for a simulated UR5e. The controller drives the end-effector toward a target and applies the obstacle modulation method from Khansari-Zadeh and Billard (2012) when obstacle avoidance is enabled. MoveIt Servo converts Cartesian velocity commands to joint motion.

The Gazebo barrier is visual only. Avoidance is handled by the DS controller. Clearance is measured for the end-effector point; it does not guarantee that every robot link clears the obstacle.

## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- MoveIt 2 and MoveIt Servo
- Universal Robots `ur_description` package

### UR5e Robot Model

This project uses the official UR5e robot description provided by Universal Robots.

Install it using:

```bash
sudo apt update
sudo apt install ros-humble-ur-description
```

Source ROS 2 and verify that the package is available:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ur_description
```

The experiment launch files use the UR5e model (`ur_type:=ur5e`). No physical robot is required; the manipulator runs in Gazebo.

For additional information, refer to the [official Universal Robots ROS 2 Description repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).

## Build

```bash
cd <PROJECT_WORKSPACE>
source /opt/ros/humble/setup.bash
colcon build --packages-select ds_table_experiment
source install/setup.bash
```
## Run the experiments

Run one scenario at a time and stop the current launch with `Ctrl+C` before starting another.

| Experiment | Scenario | Launch command |
| --- | --- | --- |
| 1 | Goal reaching without obstacle modulation | `ros2 launch ds_table_experiment experiment_1_goal_reaching.launch.py` |
| 2 | Obstacle on the direct path from A to B | `ros2 launch ds_table_experiment experiment_2_obstacle_on_path.launch.py` |
| 3 | Obstacle away from the direct path from A to B | `ros2 launch ds_table_experiment experiment_3_obstacle_off_path.launch.py` |
| 4 | Target changes while the robot is moving | `ros2 launch ds_table_experiment experiment_4_changing_target.launch.py` |
| 5 | Moving obstacle (additional experiment) | `ros2 launch ds_table_experiment experiment_2_obstacle_on_path.launch.py` |

You can disable Gazebo's GUI or RViz with `gui:=false` or `rviz:=false`. The shared launch also accepts an `experiment` folder name, for example:

```bash
ros2 launch ds_table_experiment nominal_ab_clean.launch.py \
    experiment:=experiment_2_obstacle_on_path
```

Experiment 4 starts toward target B and publishes a new target during execution. The controller accepts targets on `/new_target` as `geometry_msgs/PointStamped` messages in `base_link`.

Experiment 5 uses the same setup as Experiment 2. After launching the scenario, move the obstacle during execution using Gazebo's drag-and-drop functionality to observe how the controller reacts to changes in the obstacle position.
## Demonstration videos

The recordings for the five experiments are included in the [`videos/`](videos/) directory.

| Experiment | Recording |
| --- | --- |
| 1. Goal reaching | [Watch Experiment 1](videos/experiment_1_goal_reaching.webm) |
| 2. Obstacle on path | [Watch Experiment 2](videos/experiment_2_obstacle_on_path.webm) |
| 3. Obstacle off path | [Watch Experiment 3](videos/experiment_3_obstacle_off_path.webm) |
| 4. Changing target | [Watch Experiment 4](videos/experiment_4_changing_target.webm) |
| 5. Moving obstacle (Extra) | [Watch Experiment 5](videos/experiment_5_moving_obstacle.webm) |

## Configuration and code

Each scenario has its own Gazebo world and controller parameters under `experiments/`. Shared code and settings are organized as follows:

- `scripts/Controller.py`: target tracking, obstacle modulation, and velocity commands.
- `scripts/algorithms/nominal_ds.py`: nominal DS and obstacle modulation math.
- `scripts/plotting/visualize_results.py`: live plotting node.
- `scripts/plotting/ds_plots.py`: reusable plotting and geometry helpers.
- `config/servo_velocity.yaml`: MoveIt Servo configuration.
- `experiments/<scenario>/controller_params.yaml`: scenario-specific controller and robot start parameters.
- `experiments/<scenario>/table.world`: scenario-specific Gazebo geometry.

The controller reports the final end-effector-to-target error and minimum signed clearance from the physical box on shutdown. A negative clearance means the end-effector point entered the box. This clearance metric does not account for the robot's full body.

The automatic visualizer can be given a different output directory:

```bash
ros2 launch ds_table_experiment nominal_ab_clean.launch.py \
    visualization_output:=my_experiment_plots
```

The controller math tests can be run without ROS or a robot:

```bash
cd <PROJECT_WORKSPACE>/src/ds_table_experiment
PYTHONPATH=scripts python3 -m unittest discover -s scripts/tests
```

## Research reference

S. M. Khansari-Zadeh and A. Billard, "A Dynamical System Approach to Realtime Obstacle Avoidance," *Autonomous Robots*, 2012.

[DOI: 10.1007/s10514-012-9287-y](https://doi.org/10.1007/s10514-012-9287-y)

## Evaluation Report

[View the Technical Evaluation Report](https://drive.google.com/file/d/12V2doU9QNDBdbwo0nxecyX3v4R-22Gp0/view?usp=sharing)
