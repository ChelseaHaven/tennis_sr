"""Convert the CAN bridge's chassis vectors into planner-compatible odometry."""
import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class F407Odometry(Node):
    def __init__(self):
        super().__init__('f407_odometry')
        self.position = None
        self.velocity = None
        self.position_received_at = None
        self.velocity_received_at = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(
            Vector3Stamped, '/chassis/position', self.position_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, '/chassis/velocity', self.velocity_callback,
            qos_profile_sensor_data)
        self.create_timer(0.02, self.publish_odometry)
        self.get_logger().info(
            'Converting /chassis/position and /chassis/velocity to /odom '
            '(odom -> base_footprint)')

    def position_callback(self, message):
        values = (message.vector.x, message.vector.y, message.vector.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Ignoring non-finite chassis position')
            return
        self.position = values
        self.position_received_at = time.monotonic()

    def velocity_callback(self, message):
        values = (message.vector.x, message.vector.y, message.vector.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Ignoring non-finite chassis velocity')
            return
        self.velocity = values
        self.velocity_received_at = time.monotonic()

    def publish_odometry(self):
        if self.position is None or self.velocity is None:
            return
        now_steady = time.monotonic()
        if (now_steady - self.position_received_at > 0.2 or
                now_steady - self.velocity_received_at > 0.2):
            return

        x, y, yaw = self.position
        vx, vy, wz = self.velocity
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # The chassis velocity is reported in the robot body frame.
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        self.odom_pub.publish(message)


def main():
    rclpy.init()
    node = F407Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
