#!/usr/bin/env python3

"""Live visualization for the DS obstacle-avoidance experiment."""

from datetime import datetime
from pathlib import Path
from threading import Lock, Thread
import time

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
import rclpy
from plotting.ds_plots import (
    ControllerModel,
    draw_3d,
    draw_all_fields,
    draw_distance,
)
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class LiveVisualizer(Node):
    def __init__(self):
        super().__init__("ds_experiment_visualizer")

        defaults = {
            "planning_frame": "base_link",
            "end_effector_link": "tool0",
            "obstacle_enabled": True,
            "obstacle_size": [0.70, 0.10, 0.40],
            "obstacle_center": [0.65, 0.0, 0.60],
            "obstacle_safety_margin": 0.10,
            "obstacle_safety_factor": [1.0, 1.0, 1.0],
            "obstacle_reactivity": 1.0,
            "obstacle_tail_effect": False,
            "goal_position": [0.403, -0.363, 0.543],
            "gain": 0.75,
            "speed_limit": 0.10,
            "control_period": 0.02,
            "output_directory": "/home/sumeadh/Projects/ur5e_sorting_ws/src/ds_table_experiment/results",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.planning_frame = self.get_parameter("planning_frame").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.obstacle_enabled = bool(self.get_parameter("obstacle_enabled").value)
        self.obstacle_size = np.asarray(
            self.get_parameter("obstacle_size").value, dtype=float
        )
        self.obstacle_center = np.asarray(
            self.get_parameter("obstacle_center").value, dtype=float
        )
        self.safety_margin = float(
            self.get_parameter("obstacle_safety_margin").value
        )
        self.safety_factor = np.asarray(
            self.get_parameter("obstacle_safety_factor").value, dtype=float
        )
        self.reactivity = float(
            self.get_parameter("obstacle_reactivity").value
        )
        self.tail_effect = bool(
            self.get_parameter("obstacle_tail_effect").value
        )
        self.goal = np.asarray(
            self.get_parameter("goal_position").value, dtype=float
        )
        self.gain = float(self.get_parameter("gain").value)
        self.speed_limit = float(self.get_parameter("speed_limit").value)
        self.control_period = float(self.get_parameter("control_period").value)
        self.output_directory = Path(
            self.get_parameter("output_directory").value
        ).expanduser()

        self.lock = Lock()
        self.samples = []
        self.latest_velocity = np.zeros(3)
        self.field_dirty = True
        self.started_at = None
        self.last_field_draw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_model_states, 10
        )
        self.create_subscription(PointStamped, "/new_target", self.on_target, 10)
        self.create_subscription(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            self.on_velocity,
            10,
        )
        self.create_timer(0.1, self.sample_robot)
        self.setup_plot()

    def make_model(self, gazebo_center):
        """Same ellipsoid + same modulation function as Controller.servo_step."""
        return ControllerModel(
            self.obstacle_size, gazebo_center, self.safety_margin,
            self.safety_factor, self.reactivity, self.tail_effect,
            self.gain, self.speed_limit, self.control_period,
        )

    def setup_plot(self):
        plt.ion()
        self.figure = plt.figure("Live DS Experiment")
        grid = GridSpec(
            3, 3, figure=self.figure,
            width_ratios=(1.15, 1.0, 1.15),
            height_ratios=(1.0, 1.0, 1.0),
            wspace=0.42, hspace=0.48,
        )
        self.path_axes = self.figure.add_subplot(grid[:, 0], projection="3d")
        self.distance_axes = self.figure.add_subplot(grid[0:2, 1])
        self.gamma_axes = self.figure.add_subplot(grid[2, 1])
        self.xy_axes = self.figure.add_subplot(grid[0, 2])
        self.xz_axes = self.figure.add_subplot(grid[1, 2])
        self.yz_axes = self.figure.add_subplot(grid[2, 2])
        self.figure.set_size_inches(19, 9)
        self.figure.suptitle(
            "DS Simulation - Goal Reaching and Obstacle Avoidance", y=0.98
        )
        self.last_field_draw = time.monotonic()
        self.figure.tight_layout(rect=(0, 0, 1, 0.96))

    def on_model_states(self, message):
        try:
            index = message.name.index("experiment_obstacle")
        except ValueError:
            return
        p = message.pose[index].position
        center = np.asarray((p.x, p.y, p.z), dtype=float)
        with self.lock:
            if not np.allclose(center, self.obstacle_center, atol=1e-4):
                self.field_dirty = True
            self.obstacle_center = center

    def on_target(self, message):
        if message.header.frame_id and message.header.frame_id != self.planning_frame:
            self.get_logger().warning(
                f"Ignoring target in frame '{message.header.frame_id}'."
            )
            return
        target = np.asarray(
            (message.point.x, message.point.y, message.point.z), dtype=float
        )
        with self.lock:
            self.goal = target
            self.field_dirty = True

    def on_velocity(self, message):
        v = message.twist.linear
        with self.lock:
            self.latest_velocity = np.asarray((v.x, v.y, v.z), dtype=float)

    def sample_robot(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.end_effector_link, Time()
            ).transform
        except TransformException:
            return

        p = transform.translation
        position = np.asarray((p.x, p.y, p.z), dtype=float)
        if self.started_at is None:
            self.started_at = time.monotonic()
        with self.lock:
            elapsed = time.monotonic() - self.started_at
            self.samples.append(
                (
                    elapsed,
                    position,
                    self.latest_velocity.copy(),
                    self.obstacle_center.copy(),
                    self.goal.copy(),
                )
            )

    def update_plot(self):
        with self.lock:
            samples = list(self.samples)
            field_dirty = self.field_dirty
        if not samples:
            return

        times = np.asarray([sample[0] for sample in samples], dtype=float)
        positions = np.asarray([sample[1] for sample in samples], dtype=float)
        velocities = np.asarray([sample[2] for sample in samples], dtype=float)
        centers = np.asarray([sample[3] for sample in samples], dtype=float)
        goals = np.asarray([sample[4] for sample in samples], dtype=float)
        center = centers[-1]
        goal = goals[-1]
        model = self.make_model(center)
        gamma = model.gamma(positions)

        draw_3d(self.path_axes, model, positions, goal, gamma)
        draw_distance(self.distance_axes, self.gamma_axes, model, times, positions)

        if field_dirty or time.monotonic() - self.last_field_draw >= 1.5:
            draw_all_fields(
                (self.xy_axes, self.xz_axes, self.yz_axes), model, goal, positions
            )
            self.last_field_draw = time.monotonic()
            with self.lock:
                self.field_dirty = False

        self.figure.canvas.draw_idle()

    def save_plot(self):
        with self.lock:
            has_samples = bool(self.samples)
        if not has_samples:
            self.get_logger().error("No robot transforms received; no plot saved.")
            return
        self.update_plot()
        self.output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = self.output_directory / f"ds_visualization_{timestamp}.png"
        self.figure.savefig(output, dpi=300, bbox_inches="tight")
        self.get_logger().info(f"Saved experiment plots to {output.resolve()}")


def main(args=None):
    rclpy.init(args=args)
    node = LiveVisualizer()
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    try:
        while rclpy.ok() and plt.get_fignums():
            plt.pause(0.05)
            node.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join()
        node.save_plot()
        node.destroy_node()
        plt.close("all")


if __name__ == "__main__":
    main()
