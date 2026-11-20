#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


class AddTable(Node):
    def __init__(self):
        super().__init__("add_experiment_obstacle")
        self.declare_parameter("planning_frame")
        self.declare_parameter("obstacle_size")
        self.declare_parameter("obstacle_center")

        self.scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.finished = False
        self.timer = self.create_timer(0.5, self.add_obstacle_when_ready)

    def add_obstacle_when_ready(self):
        if not self.scene_client.service_is_ready():
            return

        self.timer.cancel()
        obstacle_size = self.get_parameter("obstacle_size").value
        obstacle_center = self.get_parameter("obstacle_center").value

        obstacle = CollisionObject()
        obstacle.header.frame_id = self.get_parameter("planning_frame").value
        obstacle.id = "experiment_obstacle"
        obstacle.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(obstacle_size)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = obstacle_center
        pose.orientation.w = 1.0
        obstacle.primitives = [box]
        obstacle.primitive_poses = [pose]

        color = ObjectColor()
        color.id = obstacle.id
        color.color = ColorRGBA(r=0.9, g=0.24, b=0.16, a=1.0)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obstacle]
        scene.object_colors = [color]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.scene_client.call_async(request).add_done_callback(self.on_obstacle_added)

    def on_obstacle_added(self, future):
        try:
            if not future.result().success:
                raise RuntimeError("MoveIt rejected the experiment obstacle.")
            self.get_logger().info("Added the red obstacle to MoveIt's scene.")
        except Exception as error:
            self.get_logger().error(f"Failed to add the red obstacle: {error}")
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = AddTable()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
