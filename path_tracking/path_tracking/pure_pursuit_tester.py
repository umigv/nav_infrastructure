"""
Example usage for a curve:
ros2 run path_tracking pure_pursuit_tester -- --radius 2 --num_points 5 --theta_start 0 --theta_end 0.3927
"""

#!/usr/bin/env python3
import math
import argparse
import rclpy
from rclpy.node import Node

from infra_interfaces.msg import FollowPathRequest, CellCoordinateMsg


class FollowPathPublisher(Node):
    def __init__(self, radius, num_points, theta_start, theta_end):
        super().__init__('follow_path_publisher')

        # Create a publisher to the /follow_path topic
        self.publisher_ = self.create_publisher(FollowPathRequest, '/follow_path', 10)

        self.get_logger().info(
            f'FollowPathPublisher started with radius={radius}, num_points={num_points}, '
            f'theta_start={theta_start}, theta_end={theta_end}'
        )

        msg = FollowPathRequest()
        msg.path = []

        # # Create a simple test path
        # msg.path = []
        # for x in range(5):
        #     cell = CellCoordinateMsg()
        #     cell.x = int(x)
        #     cell.y = 0
        #     msg.path.append(cell)

        # Generate curved path points
        for i in range(num_points + 1):
            theta = theta_start + (theta_end - theta_start) * i / num_points
            cell = CellCoordinateMsg()
            cell.x = radius * math.cos(theta)
            cell.y = radius * math.sin(theta)
            msg.path.append(cell)

        msg.resolution = 0.05  # meters per costmap cell

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published path with {len(msg.path)} points, radius={self.radius}, '
            f'theta_range=({self.theta_start:.2f}, {self.theta_end:.2f})'
        )


def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Publish a curved FollowPathRequest.')
    parser.add_argument('--radius', type=float, default=10.0, help='Arc radius in costmap cells (default: 10.0)')
    parser.add_argument('--num_points', type=int, default=30, help='Number of points in the path (default: 30)')
    parser.add_argument('--theta_start', type=float, default=0.0, help='Starting angle in radians (default: 0.0)')
    parser.add_argument('--theta_end', type=float, default=math.pi / 2, help='Ending angle in radians (default: π/2)')
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = FollowPathPublisher(
        radius=parsed_args.radius,
        num_points=parsed_args.num_points,
        theta_start=parsed_args.theta_start,
        theta_end=parsed_args.theta_end,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()