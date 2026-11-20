#!/usr/bin/env python3

from math import hypot, isfinite

import rclpy
from algorithms.nominal_ds import compute_velocity
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
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
            "goal_offset",
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
        self.servo_timer = self.create_timer(
            float(self.get_parameter("control_period").value), self.servo_step
        )
        self.servo_timer.cancel()
        self.servo_steps = 0
        self.goal: tuple[float, float] | None = None
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
        self.get_logger().info("Adding the obstacle and starting the task.")
        self.start()

    def start(self):
        self.obstacle_size = self.get_parameter("obstacle_size").value
        self.obstacle_center = self.get_parameter("obstacle_center").value
        self.goal_offset = self.get_parameter("goal_offset").value
        if (
            len(self.obstacle_size) != 3
            or len(self.obstacle_center) != 3
            or len(self.goal_offset) != 2
            or not all(isfinite(value) for value in self.obstacle_size)
            or not all(isfinite(value) for value in self.obstacle_center)
            or not all(isfinite(value) for value in self.goal_offset)
            or any(value <= 0.0 for value in self.obstacle_size)
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
            self.stop("Obstacle dimensions, goal offset, or velocity parameters are invalid.")
            return
        self.add_obstacle()

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
            self.goal = (
                position.x + self.goal_offset[0],
                position.y + self.goal_offset[1],
            )
            self.get_logger().info(
                f"Velocity-control goal B: "
                f"({self.goal[0]:.3f}, {self.goal[1]:.3f}) m."
            )
            self.stage = "servoing"
            self.servo_timer.reset()
            self.get_logger().info("Starting velocity control from A toward B.")
        except Exception as error:
            self.stop(str(error))

    def servo_step(self):
        if self.stage != "servoing" or self.goal is None:
            return

        self.servo_steps += 1
        if self.servo_steps > int(self.get_parameter("maximum_steps").value):
            self.stop("Velocity control timed out before reaching B.")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame, self.end_effector_link, Time()
            ).transform
            current = (transform.translation.x, transform.translation.y)
            distance = hypot(
                self.goal[0] - current[0],
                self.goal[1] - current[1],
            )
            if distance <= float(self.get_parameter("position_tolerance").value):
                self.publish_twist(0.0, 0.0)
                self.servo_timer.cancel()
                self.stage = "complete"
                self.get_logger().info("Velocity-controlled task reached B.")
                return

            velocity = compute_velocity(
                current,
                self.goal,
                float(self.get_parameter("gain").value),
                float(self.get_parameter("speed_limit").value),
            )
            self.publish_twist(velocity[0], velocity[1])
        except Exception as error:
            self.stop(str(error))

    def publish_twist(self, x, y):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.planning_frame
        command.twist.linear.x = x
        command.twist.linear.y = y
        self.servo_commands.publish(command)

    def stop(self, message):
        was_servoing = self.stage == "servoing"
        self.stage = "failed"
        self.get_logger().error(message)
        self.servo_timer.cancel()
        if was_servoing:
            self.publish_twist(0.0, 0.0)


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
