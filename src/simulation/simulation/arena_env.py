"""Algorithm-independent Onyx arena reset and world-state manager."""

import json
import math
import random
import re
import secrets
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode


WORLD = 'onyx_arena'
ROBOT = 'onyx'
GRID = 0.25
GRID_LIMIT = 42                 # Grid coordinates from -10.5 to +10.5 metres.
BOX_HALF = 0.2                  # Half-width of each cone's 0.4 m square base.
ROBOT_RADIUS = 1.05             # Conservative enclosing circle for Onyx.
CLEARANCE = 0.15
INFLATION = ROBOT_RADIUS + CLEARANCE + GRID * math.sqrt(2.0) / 2.0
MAX_CONES = 10                  # Keep in sync with the models in the SDF world.
OBSTACLE_COORD_LIMIT = 11.2    # Walls are at +/-12 m, inside surface.
OFFSTAGE = [(30.0 + 3.0 * (i % 4), 30.0 + 3.0 * (i // 4))
            for i in range(MAX_CONES)]


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def point_is_free(x, y, obstacles):
    # The grid is restricted to +/-10.5; the inner wall is at +/-12.
    if abs(x) > GRID_LIMIT * GRID or abs(y) > GRID_LIMIT * GRID:
        return False
    for ox, oy in obstacles:
        dx = max(abs(x - ox) - BOX_HALF, 0.0)
        dy = max(abs(y - oy) - BOX_HALF, 0.0)
        if math.hypot(dx, dy) <= INFLATION:
            return False
    return True


def connected_free_cells(obstacles):
    free = {
        (i, j)
        for i in range(-GRID_LIMIT, GRID_LIMIT + 1)
        for j in range(-GRID_LIMIT, GRID_LIMIT + 1)
        if point_is_free(i * GRID, j * GRID, obstacles)
    }
    components = []
    seen = set()
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1),
                  (1, 1), (1, -1), (-1, 1), (-1, -1))
    for first in sorted(free):
        if first in seen:
            continue
        seen.add(first)
        stack = [first]
        component = []
        while stack:
            i, j = stack.pop()
            component.append((i, j))
            for di, dj in directions:
                nxt = (i + di, j + dj)
                if nxt not in free or nxt in seen:
                    continue
                # Do not cut diagonally between two occupied cells.
                if di and dj and ((i + di, j) not in free or (i, j + dj) not in free):
                    continue
                seen.add(nxt)
                stack.append(nxt)
        components.append(sorted(component))
    return components


def make_scenario(seed, minimum, maximum):
    if seed < 0 or not (0 <= minimum <= maximum <= MAX_CONES):
        raise ValueError(f'Seed must be nonnegative and obstacle limits must satisfy 0 <= min <= max <= {MAX_CONES}.')
    rng = random.Random(seed)
    count = rng.randint(minimum, maximum)
    for _ in range(100):
        obstacles = []
        for _ in range(300):
            if len(obstacles) == count:
                break
            candidate = (round(rng.uniform(-OBSTACLE_COORD_LIMIT, OBSTACLE_COORD_LIMIT), 2),
                         round(rng.uniform(-OBSTACLE_COORD_LIMIT, OBSTACLE_COORD_LIMIT), 2))
            if any(abs(candidate[0] - x) < 1.4 and
                   abs(candidate[1] - y) < 1.4 for x, y in obstacles):
                continue
            obstacles.append(candidate)
        if len(obstacles) != count:
            continue
        components = [c for c in connected_free_cells(obstacles) if len(c) >= 2]
        rng.shuffle(components)
        for component in components:
            for _ in range(100):
                start, goal = rng.sample(component, 2)
                sx, sy = start[0] * GRID, start[1] * GRID
                gx, gy = goal[0] * GRID, goal[1] * GRID
                if math.hypot(sx - gx, sy - gy) < 2.0:
                    continue
                return {
                    'seed': seed,
                    'start': {'x': sx, 'y': sy, 'yaw': round(rng.uniform(-math.pi, math.pi), 6)},
                    'goal': {'x': gx, 'y': gy},
                    'obstacles': [{'x': x, 'y': y} for x, y in obstacles],
                }
    raise RuntimeError('Could not sample a connected scenario after 100 attempts.')


class ArenaEnv(Node):
    def __init__(self):
        super().__init__('arena_env')
        self.declare_parameter('seed', -1)
        self.declare_parameter('min_obstacles', 3)
        self.declare_parameter('max_obstacles', MAX_CONES)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('publish_world_tf', False)

        self.group = ReentrantCallbackGroup()
        self.data_lock = threading.Lock()
        self.reset_lock = threading.Lock()
        self.ground_truth = None
        self.odom = None
        self.phase = 'initial'
        self.error = ''
        self.collision = False
        self.contact_verified = False
        self.scenario = {
            'seed': None, 'start': None, 'goal': {'x': 4.0, 'y': -4.0},
            'obstacles': [{'x': 2.8, 'y': 1.7}, {'x': -2.8, 'y': -1.7},
                          {'x': -2.8, 'y': 2.5}],
        }
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/arena/goal', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/arena/robot_pose', 10)
        self.state_pub = self.create_publisher(String, '/arena/state', 10)
        self.tf_pub = TransformBroadcaster(self)

        self.gz_node = GazeboNode()
        if not self.gz_node.subscribe(
            Pose_V,
            f'/world/{WORLD}/dynamic_pose/info',
            self.pose_callback
        ):
            raise RuntimeError('Could not subscribe to Gazebo ground-truth poses.')

        self.create_subscription(Contacts, '/arena/contact', self.contact_callback,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Odometry, '/odom', self.odom_callback,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_service(Trigger, '/arena/reset', self.reset_callback,
                            callback_group=self.group)
        self.create_timer(0.2, self.publish_state, callback_group=self.group)
        self.get_logger().info('Waiting for Onyx. Stop controllers before calling /arena/reset.')

    def pose_callback(self, msg):
        for pose in msg.pose:
            if pose.name == ROBOT:
                p = pose.position
                q = pose.orientation

                yaw = yaw_from_quaternion(q)

                if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                    return

                with self.data_lock:
                    self.ground_truth = (
                        float(p.x),
                        float(p.y),
                        yaw,
                        time.monotonic()
                    )

                break

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist
        yaw = yaw_from_quaternion(q)
        if not all(math.isfinite(vv) for vv in (p.x, p.y, yaw, v.linear.x, v.angular.z)):
            return
        with self.data_lock:
            self.odom = (float(p.x), float(p.y), yaw,
                         float(v.linear.x), float(v.angular.z), time.monotonic())

    def contact_callback(self, msg):
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any(ROBOT in name.split('::') for name in names):
                with self.data_lock:
                    if self.phase != 'resetting':
                        self.contact_verified = True
                        self.collision = True
                return

    def gz_call(self, suffix, request_type, request):
        command = [
            'gz', 'service', '-s', f'/world/{WORLD}/{suffix}/blocking',
            '--reqtype', request_type, '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', request,
        ]
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        output = result.stdout + result.stderr
        if result.returncode != 0 or not re.search(r'\bdata:\s*true\b', output, re.I):
            raise RuntimeError(f'Gazebo {suffix} failed: {output.strip() or "no response"}')

    def move_model(self, name, x, y, z):
        request = (f'name: "{name}" position: {{x: {x:.6f} y: {y:.6f} z: {z:.6f}}} '
                   'orientation: {w: 1.0}')
        self.gz_call('set_pose', 'gz.msgs.Pose', request)

    def respawn(self, start):
        command = [
            'ros2', 'run', 'ros_gz_sim', 'create', '-world', WORLD,
            '-topic', '/robot_description', '-name', ROBOT,
            '-x', str(start['x']), '-y', str(start['y']), '-z', '0.05',
            '-Y', str(start['yaw']),
        ]
        start_time = time.monotonic()
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        output = result.stdout + result.stderr
        if result.returncode != 0 or '[ERROR]' in output or 'timed out' in output.lower():
            raise RuntimeError(f'Rover spawn failed: {output.strip()}')
        # ros_gz_sim create can return success even if its request failed.
        # Require fresh ground truth and odometry near the requested pose.
        deadline = time.monotonic() + 15.0
        stable_since = None
        last_ground_truth = None
        last_gt_stamp = last_odom_stamp = None
        gt_samples = odom_samples = 0
        while time.monotonic() < deadline and rclpy.ok():
            with self.data_lock:
                gt, odom = self.ground_truth, self.odom
            good = (
                gt is not None and odom is not None and
                gt[3] > start_time and odom[5] > start_time and
                time.monotonic() - gt[3] < 1.0 and
                time.monotonic() - odom[5] < 1.0 and
                math.hypot(gt[0] - start['x'], gt[1] - start['y']) < 0.4 and
                abs(normalize_angle(gt[2] - start['yaw'])) < 0.25 and
                abs(odom[3]) < 0.03 and abs(odom[4]) < 0.08
            )
            if good:
                if gt[3] != last_gt_stamp:
                    gt_samples += 1
                    last_gt_stamp = gt[3]
                if odom[5] != last_odom_stamp:
                    odom_samples += 1
                    last_odom_stamp = odom[5]
                if last_ground_truth is not None:
                    moved = math.hypot(gt[0] - last_ground_truth[0], gt[1] - last_ground_truth[1])
                    turned = abs(normalize_angle(gt[2] - last_ground_truth[2]))
                    if moved > 0.02 or turned > 0.03:
                        stable_since = None
                last_ground_truth = gt
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 1.0 and gt_samples >= 5 and odom_samples >= 5:
                    return
            else:
                stable_since = None
                last_ground_truth = None
                gt_samples = odom_samples = 0
                last_gt_stamp = last_odom_stamp = None
            time.sleep(0.1)
        raise RuntimeError('Rover did not appear and settle at the requested pose; check Gazebo and /arena/robot_pose.')

    def reset_callback(self, request, response):
        del request
        if not self.reset_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A reset is already in progress.'
            return response
        try:
            seed = int(self.get_parameter('seed').value)
            minimum = int(self.get_parameter('min_obstacles').value)
            maximum = int(self.get_parameter('max_obstacles').value)
            if seed < -1:
                raise ValueError('seed must be -1 or a nonnegative integer.')
            if seed == -1:
                seed = secrets.randbits(31)
            scenario = make_scenario(seed, minimum, maximum)
            with self.data_lock:
                gt = self.ground_truth
            if gt is None or time.monotonic() - gt[3] > 2.0:
                raise RuntimeError('No recent Gazebo ground-truth pose for onyx. Is the world running?')
            with self.data_lock:
                self.phase = 'resetting'
                self.error = ''
            self.cmd_pub.publish(Twist())
            time.sleep(0.3)
            self.gz_call('remove', 'gz.msgs.Entity', 'name: "onyx" type: MODEL')
            for i in range(MAX_CONES):
                if i < len(scenario['obstacles']):
                    position = scenario['obstacles'][i]
                    x, y = position['x'], position['y']
                else:
                    x, y = OFFSTAGE[i]
                # The cone mesh is modelled with its base at z=0.
                self.move_model(f'obstacle_{i + 1}', x, y, 0.0)
            goal = scenario['goal']
            self.move_model('goal_marker', goal['x'], goal['y'], 0.015)
            with self.data_lock:
                self.ground_truth = None
                self.odom = None
            self.respawn(scenario['start'])
            with self.data_lock:
                self.scenario = scenario
                self.collision = False
                self.phase = 'ready'
            response.success = True
            response.message = json.dumps(scenario, separators=(',', ':'))
            self.get_logger().info(f'Reset complete: {response.message}')
        except Exception as exc:
            with self.data_lock:
                self.phase = 'error'
                self.error = str(exc)
            response.success = False
            response.message = f'Reset failed: {exc}. Restart the arena if it is left partially reset.'
            self.get_logger().error(response.message)
        finally:
            self.reset_lock.release()
        return response

    def publish_state(self):
        with self.data_lock:
            gt, odom = self.ground_truth, self.odom
            scenario, phase, error = self.scenario, self.phase, self.error
            collision = self.collision if self.contact_verified else None
        now = time.monotonic()
        valid_gt = gt is not None and now - gt[3] < 1.0
        valid_odom = odom is not None and now - odom[5] < 1.0
        stamp = self.get_clock().now().to_msg()
        goal = scenario['goal']

        goal_msg = PoseStamped()
        goal_msg.header.stamp = stamp
        goal_msg.header.frame_id = 'world'
        goal_msg.pose.position.x = float(goal['x'])
        goal_msg.pose.position.y = float(goal['y'])
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

        robot = None
        distance = None
        if valid_gt:
            robot = {'x': gt[0], 'y': gt[1], 'yaw': gt[2]}
            distance = math.hypot(gt[0] - goal['x'], gt[1] - goal['y'])
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x = gt[0]
            pose.pose.position.y = gt[1]
            pose.pose.orientation.z = math.sin(gt[2] / 2.0)
            pose.pose.orientation.w = math.cos(gt[2] / 2.0)
            self.pose_pub.publish(pose)

        if (phase not in ('resetting', 'error') and valid_gt and valid_odom and
                abs(gt[3] - odom[5]) < 0.2 and
                bool(self.get_parameter('publish_world_tf').value)):
            angle = normalize_angle(gt[2] - odom[2])
            c, s = math.cos(angle), math.sin(angle)
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'odom'
            transform.transform.translation.x = gt[0] - (c * odom[0] - s * odom[1])
            transform.transform.translation.y = gt[1] - (s * odom[0] + c * odom[1])
            transform.transform.rotation.z = math.sin(angle / 2.0)
            transform.transform.rotation.w = math.cos(angle / 2.0)
            self.tf_pub.sendTransform(transform)

        tolerance = float(self.get_parameter('goal_tolerance').value)
        status = {
            'phase': phase, 'error': error, 'scenario': scenario, 'robot_world': robot,
            'distance_to_goal': distance,
            'goal_reached': None if distance is None or phase in ('resetting', 'error')
                            else distance <= tolerance,
            'collision': collision,
        }
        msg = String()
        msg.data = json.dumps(status, separators=(',', ':'))
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaEnv()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
