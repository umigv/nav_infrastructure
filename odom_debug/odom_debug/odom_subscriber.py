import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
	def __init__(self):
		super().__init__('odom_subscriber')
		self.subscription = self.create_subscription(
        	Odometry,
        	'/zed/zed_node/odom',
        	self.odom_callback,
        	10)
		self.subscription

	def odom_callback(self, msg):
		x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
		print(f'Position: x={x} y={y} z={z}')

def main():
	rclpy.init()
	node = OdomSubscriber()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


