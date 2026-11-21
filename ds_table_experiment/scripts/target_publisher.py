#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("experiment_target_publisher")
        self.declare_parameter("target_position", [0.523, -0.363, 0.543])
        self.declare_parameter("delay_seconds", 2.0)
        self.declare_parameter("planning_frame", "base_link")

        self.publisher = self.create_publisher(PointStamped, "/new_target", 10)
        self.delay = float(self.get_parameter("delay_seconds").value)
        self.position = self.get_parameter("target_position").value
        self.frame = self.get_parameter("planning_frame").value

        if len(self.position) != 3 or self.delay < 0.0:
            raise ValueError("target_position must contain 3 values and delay_seconds >= 0.")

        self.timer = self.create_timer(max(self.delay, 0.001), self.publish_target)

    def publish_target(self):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        message.point.x = float(self.position[0])
        message.point.y = float(self.position[1])
        message.point.z = float(self.position[2])
        self.publisher.publish(message)

        self.get_logger().info(
            f"Published target: ({message.point.x:.3f}, "
            f"{message.point.y:.3f}, {message.point.z:.3f}) m "
            f"in {self.frame}."
        )
        self.timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
