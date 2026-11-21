#!/usr/bin/env python3

from math import hypot, isfinite

import rclpy
from algorithms.nominal_ds import (
    cap_velocity,
    compute_velocity,
    modulate_velocity,
)
from geometry_msgs.msg import PointStamped, Pose, TwistStamped
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class Controller(Node):
    def __init__(self):
        super().__init__("nominal_ab_controller_clean")
        for name in (
            "planning_frame",
            "end_effector_link",
            "obstacle_size",
            "obstacle_center",
            "obstacle_safety_margin",
            "obstacle_safety_factor",
            "obstacle_reactivity",
            "obstacle_tail_effect",
            "goal_position",
            "gain",
            "speed_limit",
            "control_period",
            "position_tolerance",
            "maximum_steps",
        ):
            self.declare_parameter(name)

        self.planning_frame = self.get_parameter("planning_frame").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.start_servo_client = self.create_client(
            Trigger, "/servo_node/start_servo"
        )
        self.servo_commands = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.target_marker_pub = self.create_publisher(
            Marker, "/experiment/target_marker", 10
        )
        self.servo_timer = self.create_timer(
            float(self.get_parameter("control_period").value), self.servo_step
        )
        self.servo_timer.cancel()
        self.target_subscription = self.create_subscription(
            PointStamped, "/new_target", self.target_callback, 10
        )
        self.servo_steps = 0
        self.goal: tuple[float, float, float] | None = None
        self.minimum_obstacle_clearance = float("inf")
        self.final_goal_error = float("inf")
        self.stage = "waiting"
        self.timer = self.create_timer(0.5, self.start_when_ready)

    def start_when_ready(self):
        if self.stage != "waiting":
            return
        if not (
            self.scene_client.service_is_ready()
            and self.start_servo_client.service_is_ready()
            and self.servo_commands.get_subscription_count() > 0
            and self.tf_buffer.can_transform(
                self.planning_frame, self.end_effector_link, Time()
            )
        ):
            return

        self.timer.cancel()
        self.get_logger().info("Starting the target-tracking experiment.")
        self.start()

    def start(self):
        self.obstacle_size = self.get_parameter("obstacle_size").value
        self.obstacle_center = self.get_parameter("obstacle_center").value
        self.obstacle_safety_margin = self.get_parameter(
            "obstacle_safety_margin"
        ).value
        self.obstacle_safety_factor = self.get_parameter(
            "obstacle_safety_factor"
        ).value
        self.obstacle_reactivity = self.get_parameter("obstacle_reactivity").value
        self.goal_position = self.get_parameter("goal_position").value
        if (
            len(self.obstacle_size) != 3
            or len(self.obstacle_center) != 3
            or len(self.obstacle_safety_factor) != 3
            or len(self.goal_position) != 3
            or not all(isfinite(value) for value in self.obstacle_size)
            or not all(isfinite(value) for value in self.obstacle_center)
            or not all(isfinite(value) for value in self.obstacle_safety_factor)
            or not all(isfinite(value) for value in self.goal_position)
            or any(value <= 0.0 for value in self.obstacle_size)
            or not isfinite(self.obstacle_safety_margin)
            or self.obstacle_safety_margin < 0.0
            or any(value < 1.0 for value in self.obstacle_safety_factor)
            or not isfinite(self.obstacle_reactivity)
            or self.obstacle_reactivity <= 0.0
            or not all(
                isfinite(self.get_parameter(name).value)
                and self.get_parameter(name).value > 0.0
                for name in (
                    "gain",
                    "speed_limit",
                    "control_period",
                    "position_tolerance",
                )
            )
            or self.get_parameter("control_period").value >= 0.1
            or self.get_parameter("maximum_steps").value <= 0
        ):
            self.stop(
                "Obstacle dimensions, avoidance parameters, goal position, "
                "or velocity parameters are invalid."
            )
            return
        #self.add_obstacle()
        self.stage = "starting_servo"
        self.start_servo_client.call_async(
            Trigger.Request()
        ).add_done_callback(self.on_servo_started)

    # Add only the red block to MoveIt's collision scene.
    def add_obstacle(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        obstacle = CollisionObject()
        obstacle.header.frame_id = self.planning_frame
        obstacle.id = "experiment_obstacle"
        obstacle.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(self.obstacle_size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.obstacle_center
        pose.orientation.w = 1.0
        obstacle.primitives = [box]
        obstacle.primitive_poses = [pose]
        color = ObjectColor()
        color.id = obstacle.id
        color.color = ColorRGBA(r=0.9, g=0.24, b=0.16, a=1.0)
        scene.world.collision_objects = [obstacle]
        scene.object_colors = [color]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.stage = "adding_obstacle"
        self.scene_client.call_async(request).add_done_callback(
            self.on_obstacle_added
        )

    def on_obstacle_added(self, future):
        try:
            if not future.result().success:
                raise RuntimeError("MoveIt rejected the red obstacle.")
            self.stage = "starting_servo"
            self.start_servo_client.call_async(
                Trigger.Request()
            ).add_done_callback(self.on_servo_started)
        except Exception as error:
            self.stop(str(error))

    def on_servo_started(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(
                    f"MoveIt Servo failed to start: {response.message}"
                )
            self.get_logger().info("MoveIt Servo started.")
            self.start_velocity_control()
        except Exception as error:
            self.stop(str(error))

    # Start from the Gazebo-initialized pose at A.
    def start_velocity_control(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.end_effector_link, Time()
            ).transform
            position = transform.translation
            self.get_logger().info(
                f"Starting tool position: "
                f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f}) m."
            )
            self.goal = tuple(float(value) for value in self.goal_position)
            self.get_logger().info(
                f"Initial target B: "
                f"({self.goal[0]:.3f}, {self.goal[1]:.3f}, "
                f"{self.goal[2]:.3f}) m."
            )
            self.stage = "servoing"
            self.servo_timer.reset()
            self.get_logger().info("Starting velocity control from A toward B.")
        except Exception as error:
            self.stop(str(error))

    def target_callback(self, message: PointStamped):
        if message.header.frame_id and message.header.frame_id != self.planning_frame:
            self.get_logger().error(
                f"Ignoring target in frame '{message.header.frame_id}'. "
                f"Expected '{self.planning_frame}'."
            )
            return

        target = (
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        )
        if not all(isfinite(value) for value in target):
            self.get_logger().error("Ignoring target containing non-finite values.")
            return

        previous_goal = self.goal
        self.goal = target
        self.servo_steps = 0
        label = "C" if previous_goal is not None else "initial"
        self.get_logger().info(
            f"Received new target {label}: "
            f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) m."
        )
        self.publish_target_marker(target)

    def publish_target_marker(self, target):
        """Show the latest commanded target as a green sphere in RViz."""
        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "experiment_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = target[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.target_marker_pub.publish(marker)

    def obstacle_clearance(self, position):
        """Signed distance from the tool point to the physical box surface."""
        half = tuple(float(size) / 2.0 for size in self.obstacle_size)
        offset = tuple(
            abs(float(value) - float(center)) - extent
            for value, center, extent in zip(
                position, self.obstacle_center, half
            )
        )
        outside = hypot(*(max(value, 0.0) for value in offset))
        inside = min(max(offset), 0.0)
        return outside + inside

    def servo_step(self):
        if self.stage != "servoing" or self.goal is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.end_effector_link, Time()
            ).transform
            current = (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            )

            clearance = self.obstacle_clearance(current)
            self.minimum_obstacle_clearance = min(
                self.minimum_obstacle_clearance, clearance
            )

            distance = hypot(
                self.goal[0] - current[0],
                self.goal[1] - current[1],
                self.goal[2] - current[2],
            )
            self.final_goal_error = distance

            if distance <= float(self.get_parameter("position_tolerance").value):
                # Keep the timer alive: hold here until another target arrives.
                self.publish_twist(0.0, 0.0, 0.0)
                return

            self.servo_steps += 1
            if self.servo_steps > int(self.get_parameter("maximum_steps").value):
                self.stop("Velocity control timed out before reaching the current target.")
                return

            velocity = compute_velocity(
                current,
                self.goal,
                float(self.get_parameter("gain").value),
                float(self.get_parameter("speed_limit").value),
            )

            # The experiment uses the current Z-stretched ellipsoid model.
            top_of_box = self.obstacle_center[2] + (self.obstacle_size[2] / 2.0)
            obstacle_semi_axes = (
                (3.0 ** 0.5) * self.obstacle_size[0] / 2.0
                + self.obstacle_safety_margin,
                (3.0 ** 0.5) * self.obstacle_size[1] / 2.0
                + self.obstacle_safety_margin,
                top_of_box + self.obstacle_safety_margin,
            )
            obs_center_math = (
                self.obstacle_center[0],
                self.obstacle_center[1],
                0.0,
            )

            velocity, _ = modulate_velocity(
                current,
                velocity,
                obs_center_math,
                obstacle_semi_axes,
                safety_factor=self.obstacle_safety_factor,
                reactivity=self.obstacle_reactivity,
                tail_effect=self.get_parameter("obstacle_tail_effect").value,
            )
            velocity = cap_velocity(
                velocity,
                float(self.get_parameter("speed_limit").value),
            )
            self.publish_twist(velocity[0], velocity[1], velocity[2])
        except Exception as error:
            self.stop(str(error))

    def publish_twist(self, x, y, z):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.planning_frame
        command.twist.linear.x = x
        command.twist.linear.y = y
        command.twist.linear.z = z
        self.servo_commands.publish(command)

    def report_metrics(self):
        if self.final_goal_error != float("inf"):
            self.get_logger().info(
                f"Final goal error: {self.final_goal_error:.4f} m"
            )
        else:
            self.get_logger().info("Final goal error: unavailable (no servo feedback).")

        if self.minimum_obstacle_clearance != float("inf"):
            self.get_logger().info(
                "Minimum tool-point clearance from physical obstacle: "
                f"{self.minimum_obstacle_clearance:.4f} m"
            )
        else:
            self.get_logger().info("Minimum obstacle clearance: unavailable.")

    def destroy_node(self):
        if self.stage == "servoing":
            self.publish_twist(0.0, 0.0, 0.0)
        self.report_metrics()
        return super().destroy_node()

    def stop(self, message):
        was_servoing = self.stage == "servoing"
        self.stage = "failed"
        self.get_logger().error(message)
        self.servo_timer.cancel()
        if was_servoing:
            self.publish_twist(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
