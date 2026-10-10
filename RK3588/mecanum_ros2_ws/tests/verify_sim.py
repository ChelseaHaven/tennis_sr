#!/usr/bin/env python3
"""Integration check against a running, isolated simulation (no real robot)."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from fast_fly_interfaces.msg import TrackTraj


def main():
    rclpy.init()
    node = Node('verify_mecanum_sim')
    latest = {}
    for topic, cls in [('/odom', Odometry), ('/joint_states', JointState),
                       ('/clock', Clock), ('/track_traj', TrackTraj), ('/planed_path', Path)]:
        node.create_subscription(cls, topic, lambda msg, key=topic: latest.update({key: msg}), 10)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    point_pub = node.create_publisher(PointStamped, '/clicked_point', 10)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

    def wait(predicate, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('Timed out; received topics: ' + ', '.join(latest))

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        wait(lambda: all(k in latest for k in ['/odom', '/joint_states', '/clock']))
        assert len(latest['/joint_states'].name) == 4, 'Expected four wheel joints'
        wait(lambda: cmd_pub.get_subscription_count() > 0 and goal_pub.get_subscription_count() > 0)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y = -0.4, -0.4
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            cmd_pub.publish(cmd)
            spin_for(0.04)
        spin_for(0.04)
        v = latest['/odom'].twist.twist.linear
        assert v.x < -0.2 and v.y < -0.2, f'Negative/lateral velocity failed: {v}'
        lateral_velocity = [v.x, v.y]
        # Stop publishing: the physics plugin must brake after its 0.25s timeout.
        spin_for(1.3)
        v = latest['/odom'].twist.twist.linear
        assert math.hypot(v.x, v.y) < 0.03, 'Command timeout did not stop the base'
        start = latest['/odom'].pose.pose.position
        point = PointStamped()
        point.header.frame_id = 'odom'
        point.point.x, point.point.y = start.x + 1.0, start.y
        point_pub.publish(point)
        spin_for(0.3)
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x, goal.pose.position.y = start.x + 1.0, start.y + 1.0
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        wait(lambda: '/track_traj' in latest and '/planed_path' in latest, 30)
        traj = latest['/track_traj']
        assert len(traj.position) == len(traj.dt) + 1
        assert math.hypot(traj.velocity[-1].x, traj.velocity[-1].y) < 1e-5
        assert all(math.isfinite(dt) and dt > 0 for dt in traj.dt)
        total = sum(traj.dt)
        # Simulation may run below real time; wait on its clock, with a wall bound.
        begin_sim = latest['/clock'].clock
        begin = begin_sim.sec + begin_sim.nanosec * 1e-9
        wait(lambda: latest['/clock'].clock.sec + latest['/clock'].clock.nanosec*1e-9 > begin + total + 3,
             max(30, total * 5 + 10))
        p = latest['/odom'].pose.pose.position
        v = latest['/odom'].twist.twist.linear
        error = math.hypot(p.x - goal.pose.position.x, p.y - goal.pose.position.y)
        speed = math.hypot(v.x, v.y)
        assert error < 0.5, f'Goal error too large: {error:.3f}m'
        assert speed < 0.03, f'Robot did not stop: {speed:.3f}m/s'
        print(json.dumps({'status': 'PASS', 'negative_body_velocity': lateral_velocity,
                          'segments': len(traj.dt), 'duration_s': total,
                          'goal_error_m': error, 'final_speed_m_s': speed}, indent=2))
    finally:
        cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
