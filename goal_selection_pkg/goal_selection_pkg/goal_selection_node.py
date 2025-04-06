import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from map_interfaces.srv import InflationGrid
from infra_interfaces.action import NavigateToGoal
from infra_interfaces.msg import CellCoordinateMsg
from geometry_msgs.msg import Pose
import numpy as np
import time
from goal_selection_pkg.goal_selection_algo import *


# Function to check if a position is valid

class GoalSelectionService(Node):
    def __init__(self):
        print("Goal Selection Node INIT")
        super().__init__('goal_selection_node')

        self.navigate_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')

        testingIntercept = True
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        if not testingIntercept:
            self.odom_sub = self.create_subscription(
                Odometry, 
                '/odom', 
                self.odom_callback, 
                qos_profile)
            self.current_orientation = None

        self.gps_coord_sub = self.create_subscription(
            NavSatFix, 
            '/gps_coords', 
            self.gps_coord_callback, 
            qos_profile)

        self.inflation_client = self.create_client(InflationGrid, 'inflation_grid_service')

        self.navigation_timer = self.create_timer(0.5, self.send_inflation_request)

    def odom_callback(self, msg):
        self.current_orientation = msg.pose.pose.orientation
        self.get_logger().info(f"Current orientation: {self.current_orientation}")

    def gps_coord_callback(self, msg):
        self.get_logger().info(f"GPS Coordinates received: {msg.latitude}, {msg.longitude}")

    def restart_navigation(self):
        self.navigation_timer.reset()

    def send_inflation_request(self):
        """
        Entry point into the navigation stack; begins the following process: 
        1. Call inflation_grid_service to get the costmap
        2. Execute goal selection algorithm to calculate a new goal
        3. Call the navigate_to_goal action to move the robot to the goal
        4. When goal is reached or failure occurs, repeat from step 1
        """
        # Will be restarted when navigation to a goal succeeds or an error occurs 
        self.navigation_timer.cancel() 

        while not self.inflation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for inflation_grid_service...')

        self.get_logger().info("Sending inflation_grid_service request")
        self.req = InflationGrid.Request()
        future = self.inflation_client.call_async(self.req)
        future.add_done_callback(self.inflation_response_callback)

    def inflation_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Received inflation grid response')
            starting_pose, new_goal, my_occgrid = self.goal_testing_wrapper(response)
            self.get_logger().info(f"Sending Starting Pose: {starting_pose} and Goal: {new_goal}")
            self.send_navigate_goal(starting_pose[::-1], new_goal[::-1], my_occgrid)
        except Exception as e:
            self.get_logger().error(f'Exception while calling service: {e}')
            self.restart_navigation()

    def goal_testing_wrapper(self, grid_msg):
        robot_pose_x, robot_pose_y = grid_msg.robot_pose_x, grid_msg.robot_pose_y
        return (robot_pose_x, robot_pose_y), (10 ,78), grid_msg.occupancy_grid

    def goal_selection_wrapper(self, grid_msg):
       
        print("Started goal_selection ")

        robot_pose_x, robot_pose_y = grid_msg.robot_pose_x, grid_msg.robot_pose_y
        matrix = np.array(grid_msg.occupancy_grid.data).reshape((grid_msg.occupancy_grid.info.height, grid_msg.occupancy_grid.info.width))
        matrix = np.flipud(matrix)
        # start_bfs = (47, 78)
        # robot_pose = (55, 78)
        start_bfs_factor = 8 # we don't want to start the search from the robot's position (because its in unknown space) 
        # so shift the starting node this much "up"
        # print("Trying to visualize cost map")
        # visualize_cost_map(matrix)
        # print("Finished  to visualize cost map")

        node_using_angle = False # boolean to say whether to use the angle to waypoint in the cost function, set to false for now

        print(np.sum(matrix))
        print(matrix.shape)


        np.set_printoptions(threshold=np.inf,linewidth=1000)

        # Why do we need to have a seperate start_bfs and robot_pose you may ask?
        # if you think about it, the robot_pose is in unknown space from CV's perspective
        # So if we bfs from the robot_pose, the algo won't work 
        # (you won't add any neighbors to the queue in the first iteration)
        # Buuuuuut if we do angle calculations from the start_bfs, we could get skewed results 
        # Sincelry,
        # Maaz 



        # Directions: Vertical, Horizontal, Diagonal
        directions = [(-1, 0),   # Up
                      (-1, -1),  # Up-left (diagonal)
                      (-1, 1),   # Up-right (diagonal)
                      (0, -1),   # Left
                      (0, 1),
                      (1,0),
                      (1,1),
                      (1,-1)]  

        # directions = [(-1, -1),  # Up-left (diagonal)
        #             (-1, 1),
        #             (1,1),
        #             (1,-1)]   # Right


        start_bfs = (robot_pose_x - start_bfs_factor, robot_pose_y)  # Example offset for BFS start()
        min_cost_cell, min_cost  = bfs_with_cost((robot_pose_x, robot_pose_y), matrix, start_bfs, directions, using_angle=node_using_angle)
        print("Cell with Minimum Cost: ", min_cost_cell, "Minimum Cost: ", min_cost)

        return (robot_pose_x, robot_pose_y), min_cost_cell, grid_msg.occupancy_grid

    def send_navigate_goal(self, starting_pose, new_goal, my_occgrid):
        """ Sends a goal to the NavigateToGoal action and waits for the result or feedback condition """

        self.starting_pose = starting_pose[::-1]  # Reverse the order for the action server
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToGoal action server not available!")
            self.restart_navigation()
            return

        goal_msg = NavigateToGoal.Goal()
        goal_msg.costmap = my_occgrid
        goal_msg.start = CellCoordinateMsg(x=starting_pose[0], y=starting_pose[1])
        goal_msg.goal = CellCoordinateMsg(x=new_goal[0], y=new_goal[1])

        self.get_logger().info(f"Sending goal: start={starting_pose}, goal={new_goal}")

        send_goal_future = self.navigate_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.navigate_to_goal_acceptance_callback)
    
    def feedback_callback(self, feedback_msg):
        """ Process feedback from the action server """
        pose = feedback_msg.feedback.distance_from_start
        self.get_logger().info(f"Feedback received: Pose({pose.position.x}, {pose.position.y})")

        # Stop if the pose is within 1.0 of the starting pose
        if abs(pose.position.x - self.starting_pose[0]) <= 1.0 and abs(pose.position.y - self.starting_pose[1]) <= 1.0:
            self.get_logger().info("Robot is within 1.0 of starting position, stopping...")
            self.navigate_client.cancel_goal_async(self.goal_handle)

    def navigate_to_goal_acceptance_callback(self, future):
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('navigate_to_goal action goal rejected')
                self.restart_navigation()
                return
                
            self.get_logger().info('navigate_to_goal action goal accepted')
            
            future = goal_handle.get_result_async()
            future.add_done_callback(self.navigate_to_goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'Exception in navigate_to_goal_acceptance_callback: {e}')
            self.restart_navigation()

    def navigate_to_goal_result_callback(self, future):
        try:
            navigation_result = future.result().success
            if navigation_result:
                self.get_logger().info('Navigation to goal succeeded, continuing to next goal')
            else:
                self.get_logger().info('Navigation to goal failed, retrying')
        except Exception as e:
            self.get_logger().error(f'Exception in navigate_to_goal_result_callback: {e}')

        self.restart_navigation()

def main():
    rclpy.init()
    node = GoalSelectionService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
