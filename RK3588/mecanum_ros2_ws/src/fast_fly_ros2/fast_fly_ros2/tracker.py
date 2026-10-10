"""Nearest-knot velocity feedforward, with odometry timeout stopping."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from fast_fly_interfaces.msg import TrackTraj


class Tracker(Node):
    def __init__(self):
        super().__init__('track')
        self.declare_parameter('lookahead', 2)
        self.lookahead = max(1, int(self.get_parameter('lookahead').value))
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(TrackTraj, '/track_traj', self.load, 1)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(Bool, '/planning', self.planning, 10)
        self.positions = self.velocities = None
        self.last_odom = time.monotonic()
        self.create_timer(0.1, self.watchdog)

    def planning(self, msg):
        if msg.data:
            self.positions = None
            self.pub.publish(Twist())

    def load(self, msg):
        n = len(msg.position)
        if n < 2 or len(msg.velocity) != n or len(msg.dt) != n - 1:
            self.get_logger().error('Invalid trajectory array lengths')
            return
        positions = np.array([[p.x, p.y] for p in msg.position])
        velocities = np.array([[v.x, v.y] for v in msg.velocity])
        if not np.isfinite(positions).all() or not np.isfinite(velocities).all() or not all(
                math.isfinite(dt) and dt > 0 for dt in msg.dt):
            self.get_logger().error('Invalid trajectory values')
            return
        self.positions, self.velocities = positions, velocities
        self.index = 0
        self.get_logger().info(f'Loaded {n} knots')

    def odom(self, msg):
        self.last_odom = time.monotonic()
        if self.positions is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        # Forward-only local search avoids jumping branches at path crossings.
        end = min(len(self.positions), self.index + 12)
        self.index += int(np.argmin(np.linalg.norm(
            self.positions[self.index:end] - [p.x, p.y], axis=1)))
        k = min(self.index + self.lookahead, len(self.positions) - 1)
        vx, vy = self.velocities[k]
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        cmd = Twist()
        cmd.linear.x = math.cos(yaw)*vx + math.sin(yaw)*vy
        cmd.linear.y = -math.sin(yaw)*vx + math.cos(yaw)*vy
        self.pub.publish(cmd)

    def watchdog(self):
        if time.monotonic() - self.last_odom > 0.3:
            self.positions = None
            self.pub.publish(Twist())


def main():
    rclpy.init()
    node = Tracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
