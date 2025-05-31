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
from goal_selection.goal_selection_algo import *
from goal_selection.waypoint_manager import GPSWaypointManager
from scipy.spatial.transform import Rotation
from goal_selection.waypoint_manager import GPSCoordinate

class RobotPose:
    def __init__(self, x: float, y: float, yaw: float = 0.0):
        """
        x and y are in meters, yaw is in radians
        """
        self.x = x
        self.y = y
        self.yaw = yaw

    def __repr__(self):
        return f"(x={self.x}, y={self.y}, yaw={self.yaw})"

class GoalSelectionNode(Node):
    def __init__(self):
        super().__init__('goal_selection_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('gps_coords_topic', '/gps_coords')
        self.declare_parameter('navigation_retry_frequency', 0.5)
        self.declare_parameter('waypoints_file_name', "waypoints.json")

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        gps_coords_topic = self.get_parameter('gps_coords_topic').get_parameter_value().string_value
        self.navigation_retry_frequency = self.get_parameter('navigation_retry_frequency').get_parameter_value().double_value
        waypoints_file_name = self.get_parameter('waypoints_file_name').get_parameter_value().string_value
        
        self.get_logger().info("Initializing goal selection node with following parameters: ")
        self.get_logger().info(f"\todom_topic: {odom_topic}")
        self.get_logger().info(f"\tgps_coords_topic: {gps_coords_topic}")
        self.get_logger().info(f"\tnavigation_retry_frequency: {self.navigation_retry_frequency}")
        self.get_logger().info(f"\twaypoints_file_name: {waypoints_file_name}")

        self.waypoints_manager = GPSWaypointManager(waypoints_file_name, self.get_logger())
        self.curr_gps_waypoint = self.waypoints_manager.get_next_waypoint()
        self.get_logger().info(f"First GPS waypoint: {self.curr_gps_waypoint}")
        self.curr_pose = None
        self.curr_gps = None

        testingIntercept = False
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        if not testingIntercept:
            self.odom_sub = self.create_subscription(
                Odometry, 
                odom_topic, 
                self.odom_callback, 
                qos_profile)

        self.gps_coord_sub = self.create_subscription(
            NavSatFix, 
            gps_coords_topic, 
            self.gps_coord_callback, 
            qos_profile)

        self.inflation_client = self.create_client(InflationGrid, 'inflation_grid_service')
        self.navigate_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        self.navigation_timer = self.create_timer(0.5, self.send_inflation_request)

    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        quat_scipy = [quat.x, quat.y, quat.z, quat.w]
        r = Rotation.from_quat(quat_scipy)
        euler_angles = r.as_euler('xyz')
        curr_yaw = euler_angles[2] 
        self.curr_pose = RobotPose(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=curr_yaw
        )
        # self.get_logger().info(f"Current pose: {self.curr_pose}")

    def gps_coord_callback(self, msg):
        self.curr_gps = GPSCoordinate(
            lat=msg.latitude,
            lon=msg.longitude
        )
        # self.get_logger().info(f"Current GPS coordinate: {self.curr_gps}")

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
            starting_pose, new_goal, my_occgrid, goal_is_waypoint = self.goal_selection_wrapper(response)
            self.get_logger().info(f"Sending Starting Pose: {starting_pose} and Goal: {new_goal}")
            self.send_navigate_goal(starting_pose[::-1], new_goal[::-1], my_occgrid, goal_is_waypoint)
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
        # matrix = np.flipud(matrix)
        # start_bfs = (47, 78)
        # robot_pose = (55, 78)
        start_bfs_factor = 15 # we don't want to start the search from the robot's position (because its in unknown space) 
        # so shift the starting node this much "up"
        # print("Trying to visualize cost map")
        visualize_cost_map(matrix)
        # print("Finished  to visualize cost map")

        node_using_angle = False # boolean to say whether to use the angle to waypoint in the cost function, set to false for now

        print(f"Sum of costmap: {np.sum(matrix)}")
        print(f"Shape of costmap: {matrix.shape}")


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
        print( "MMMM robot pose: ", robot_pose_x, robot_pose_y)

        #this bfs should check if waypoint is in the provided matrix.
        start_bfs = (robot_pose_x + start_bfs_factor, robot_pose_y)  # Example offset for BFS start()
        min_cost_cell, min_cost, cell_is_waypoint  = bfs_with_cost((robot_pose_x, robot_pose_y), 
                                                 matrix, 
                                                 start_bfs, 
                                                 directions, 
                                                 current_gps = self.curr_gps, 
                                                 goal_gps = self.curr_gps_waypoint, 
                                                 robot_orientation = self.curr_pose.yaw, 
                                                 using_angle=node_using_angle)
        print("Cell with Minimum Cost: ", min_cost_cell, "Minimum Cost: ", min_cost)
        print("WIDTH  ", grid_msg.occupancy_grid.info.width, "HEIGHT ", grid_msg.occupancy_grid.info.height)
        grid_msg.occupancy_grid.info.resolution = 0.05

        return (robot_pose_y , robot_pose_x ), (min_cost_cell[1], min_cost_cell[0]), grid_msg.occupancy_grid, cell_is_waypoint


    def send_navigate_goal(self, starting_pose, new_goal, my_occgrid, goal_is_waypoint):
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

        def navigate_to_goal_acceptance_callback(future):
            self.navigate_to_goal_acceptance_callback(future, goal_is_waypoint)

        send_goal_future.add_done_callback(navigate_to_goal_acceptance_callback)
    
    def feedback_callback(self, feedback_msg):
        """ Process feedback from the action server """
        pose = feedback_msg.feedback.distance_from_start
        self.get_logger().info(f"Feedback received: Pose({pose.position.x}, {pose.position.y})")

        # Stop if the pose is within 1.0 of the starting pose
        if abs(pose.position.x - self.starting_pose[0]) <= 1.0 and abs(pose.position.y - self.starting_pose[1]) <= 1.0:
            self.get_logger().info("Robot is within 1.0 of starting position, stopping...")
            self.navigate_client.cancel_goal_async(self.goal_handle)

    def navigate_to_goal_acceptance_callback(self, future, goal_is_waypoint):
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('navigate_to_goal action goal rejected')
                self.restart_navigation()
                return
                
            self.get_logger().info('navigate_to_goal action goal accepted')
            
            future = goal_handle.get_result_async()

            def navigate_to_goal_result_callback(future):
                self.navigate_to_goal_result_callback(future, goal_is_waypoint)

            future.add_done_callback(navigate_to_goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'Exception in navigate_to_goal_acceptance_callback: {e}')
            self.restart_navigation()

    def navigate_to_goal_result_callback(self, future, goal_is_waypoint):
        try:
            navigation_result = future.result().result.success
            if navigation_result:
                self.get_logger().info('Navigation to goal succeeded, continuing to next goal')
                if goal_is_waypoint:
                    self.curr_gps_waypoint = self.waypoints_manager.get_next_waypoint()
                    self.get_logger().info(f'Reached waypoint, beginning navigation to next waypoint: {self.curr_gps_waypoint}')
                    if self.curr_gps_waypoint is None:
                        self.get_logger().info('No more waypoints available, stopping navigation')
                        return
            else:
                self.get_logger().info('Navigation to goal failed, retrying')
        except Exception as e:
            self.get_logger().error(f'Exception in navigate_to_goal_result_callback: {e}')
        #add a timer to wait before restarting navigation
        time.sleep(self.navigation_retry_frequency)
        self.restart_navigation()

def main():
    rclpy.init()
    node = GoalSelectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
