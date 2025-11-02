import rclpy
from rclpy.node import Node

# Import your message types
from infra_interfaces.msg import FollowPathRequest, CellCoordinateMsg

class FollowPathPublisher(Node):
    def __init__(self):
        super().__init__('pure_pursuit_tester')

        # Create a publisher to the /follow_path topic
        self.publisher_ = self.create_publisher(FollowPathRequest, '/follow_path', 10)

        self.get_logger().info('FollowPathPublisher node has started.')

        msg = FollowPathRequest()

        # Create a simple test path
        msg.path = []
        for y in range(41):
            cell = CellCoordinateMsg()
            cell.x = 0
            cell.y = int(y)
            msg.path.append(cell)

        # Set the map resolution (for example, 0.05 m/cell)
        msg.resolution = 0.05

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published FollowPathRequest with {len(msg.path)} points and resolution {msg.resolution:.2f}')
        

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()