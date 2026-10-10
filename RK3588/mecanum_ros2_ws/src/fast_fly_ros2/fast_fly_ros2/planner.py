"""RViz waypoint input and the original two-stage CasADi/IPOPT optimizer."""
from concurrent.futures import ThreadPoolExecutor
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from fast_fly_interfaces.msg import TrackTraj
from .gates.gates import Gates
from .mecanum_planner import MecanumModel, MecanumWayPointOpt, cal_Ns_2d


def solve(start, waypoints, v_max, a_max, spacing, dt0):
    gates = Gates()
    for x, y in waypoints:
        gates.add_gate([x, y, 0.0])
    counts = cal_Ns_2d(gates, spacing, False, start[:2])
    if sum(counts) > 1500:
        raise ValueError('Trajectory exceeds 1500 knots; use fewer waypoints')
    opt = MecanumWayPointOpt(MecanumModel(v_max, a_max), gates._N, counts, False)
    opt.define_opt()
    opt.define_opt_t()
    points = np.asarray(waypoints).flatten()
    opt.solve_opt(start, points, np.full(gates._N, dt0))
    result = opt.solve_opt_t(start, points)['x'].full().flatten()
    states = result[:4 * opt._Herizon].reshape(-1, 4)
    intervals = np.repeat(result[-gates._N:], counts)
    # Include the initial state. dt[i] describes initial/knot i -> knot i+1.
    return np.vstack([start, states]), intervals


class Planner(Node):
    def __init__(self):
        super().__init__('plan_interactive')
        for name, default in [('v_max', 4.5), ('a_max', 3.0),
                              ('l_per_n', 0.25), ('dt0', 0.3)]:
            self.declare_parameter(name, default)
        self.config = [float(self.get_parameter(k).value)
                       for k in ['v_max', 'a_max', 'l_per_n', 'dt0']]
        if not all(np.isfinite(v) and v > 0 for v in self.config):
            raise ValueError('Planner limits, spacing and dt0 must be positive')
        self.traj_pub = self.create_publisher(TrackTraj, 'track_traj', 1)
        self.path_pub = self.create_publisher(Path, 'planed_path', 1)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, 'plan_markers', retained)
        self.busy_pub = self.create_publisher(Bool, 'planning', 1)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal, 10)
        # Keep the former topic available for existing clients.
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal, 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.state = None
        self.waypoints = []
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.create_timer(0.05, self.check_result)
        self.get_logger().info('Publish Point adds waypoints; 2D Goal Pose plans to the goal (odom frame).')

    def odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.state = np.array([p.x, p.y, np.cos(yaw)*v.x - np.sin(yaw)*v.y,
                               np.sin(yaw)*v.x + np.cos(yaw)*v.y])

    def valid(self, frame, x, y):
        if frame != 'odom' or not np.isfinite([x, y]).all() or max(abs(x), abs(y)) > 19.5:
            self.get_logger().warning('Use odom frame and finite coordinates within ±19.5 m.')
            return False
        if self.future is not None:
            self.get_logger().warning('Planning in progress; wait for the result.')
            return False
        return True

    def clicked(self, msg):
        if self.valid(msg.header.frame_id, msg.point.x, msg.point.y):
            self.waypoints.append([msg.point.x, msg.point.y])
            self.markers()

    def goal(self, msg):
        p = msg.pose.position
        if not self.valid(msg.header.frame_id, p.x, p.y):
            return
        if self.state is None:
            self.get_logger().warning('Waiting for /odom from Gazebo.')
            return
        points = self.waypoints + [[p.x, p.y]]
        self.busy_pub.publish(Bool(data=True))
        self.started = time.monotonic()
        self.future = self.pool.submit(solve, self.state.copy(), points, *self.config)

    def check_result(self):
        if self.future is None or not self.future.done():
            return
        try:
            states, intervals = self.future.result()
            traj, path = TrackTraj(), Path()
            path.header.frame_id = 'odom'
            path.header.stamp = self.get_clock().now().to_msg()
            traj.start_time = path.header.stamp
            traj.dt = intervals.astype(float).tolist()
            for x, y, vx, vy in states:
                pos = Point(x=float(x), y=float(y), z=0.0)
                traj.position.append(pos)
                traj.velocity.append(Vector3(x=float(vx), y=float(vy), z=0.0))
                traj.orientation.append(Quaternion(w=1.0))
                traj.angular.append(Vector3())
                pose = PoseStamped(header=path.header)
                pose.pose.position = pos
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.traj_pub.publish(traj)
            self.path_pub.publish(path)
            self.waypoints.clear()
            self.markers()
            self.get_logger().info(f'Planned {len(intervals)} segments in '
                                   f'{time.monotonic()-self.started:.2f}s; duration {sum(intervals):.2f}s')
        except Exception as exc:
            self.get_logger().error(f'Planning failed: {exc}')
        finally:
            self.future = None
            self.busy_pub.publish(Bool(data=False))

    def markers(self):
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'odom'
        array = MarkerArray(markers=[clear])
        for i, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = 'clicked_waypoints', i
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position = Point(x=x, y=y, z=0.08)
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=0.15, y=0.15, z=0.15)
            marker.color.r, marker.color.a = 0.9, 1.0
            array.markers.append(marker)
        self.marker_pub.publish(array)


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
