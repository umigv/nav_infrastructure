import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from infra_interfaces.action import FollowPath  # Import the action
import math
import threading
import time


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_lookahead')
        self.get_logger().info('Pure Pursuit Node started.')

        # Parameters
        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.4
        self.lookahead_distance = 0.3

        # State
        self.path = []
        self.pose = None
        self.reached_goal = False

        # Use ReentrantCallbackGroup for concurrent execution
        self.cb_group = ReentrantCallbackGroup()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.create_timer(0.1, self.control_loop, callback_group=self.cb_group)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received a new path from action client.')
        self.path = [(p.x, p.y) for p in goal_handle.request.path]
        self.reached_goal = False

        # Wait for the goal to be reached in a non-blocking loop
        while not self.reached_goal and rclpy.ok():
            time.sleep(0.05)

        goal_handle.succeed()
        result = FollowPath.Result()
        return result

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(ori)
        self.pose = (pos.x, pos.y, yaw)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_lookahead_point(self):
        if self.pose is None or not self.path:
            return None

        x, y, yaw = self.pose

        for gx, gy in self.path:
            dx = gx - x
            dy = gy - y

            # Transform to robot's frame
            local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            dist = math.hypot(local_x, local_y)

            if local_x > 0.05 and dist >= self.lookahead_distance:
                return local_x, local_y

        self.reached_goal = True
        return None

    def control_loop(self):
        if self.pose is None:
            return

        local_point = self.find_lookahead_point()

        if local_point is None:
            self.cmd_pub.publish(Twist())
            if self.reached_goal:
                self.get_logger().info('Path completed.')
            else:
                self.get_logger().info('Waiting for valid lookahead point.')
            return

        local_x, local_y = local_point
        curvature = 2 * local_y / (local_x**2 + local_y**2)
        dist = math.hypot(local_x, local_y)

        linear = min(self.max_linear_speed, dist)
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, linear * curvature))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()

    # MultiThreadedExecutor allows concurrent execution of callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()