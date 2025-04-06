
import numpy as np
import numpy as np
import random
from collections import deque
import math
import matplotlib.pyplot as plt


def is_valid_move(position, matrix, visited):  

    y, x = position
    rows, cols = matrix.shape
    return 0 <= y < rows and 0 <= x < cols and matrix[y, x] == 1 and position not in visited

# assumes radians angles
def get_angle_difference(to_angle, from_angle):
    delta = to_angle - from_angle
    delta = (delta + math.pi) % (2 * math.pi) - math.pi
    return delta

def find_desired_heading( cur_gps, goal_gps, orientation):
    # lat long degress to meter ratio for the state of Michigan
    latitude_length=111086.2 
    longitude_length=81978.2 
        
    delta_lat = goal_gps[0] - cur_gps[0]
    delta_lon = cur_gps[1] - goal_gps[1]
    north_m = delta_lat * latitude_length
    west_m = delta_lon * longitude_length
    
    # desired_heading_global = math.atan2(west_m, north_m)
    desired_heading_x = math.cos(orientation) * west_m + math.sin(orientation) * north_m
    desired_heading_y = -math.sin(orientation) * west_m + math.cos(orientation) * north_m
    desired_heading_global = math.atan2(desired_heading_y, desired_heading_x) # - math.pi
    return desired_heading_global

def get_angle_to_goal_pentaly(canidate_node, real_robot_pos, orientation, desired_heading_global):
    y,x = canidate_node
    outside_point_y, outside_point_x = real_robot_pos
    dx = x - outside_point_x
    dy = y - outside_point_y
    cell_dir_local = math.atan2(dy, dx)  

    global_cell_dir = orientation + cell_dir_local + math.pi * 0.5

    heading_error = abs(get_angle_difference(desired_heading_global, global_cell_dir))
    heading_error_deg = math.degrees(heading_error)
    return heading_error_deg


#TODO combine first 3 args into a tuple

#checks if the waypoint is in the CV frame. basically just the find_desired_heading, but with some extra math
def waypoint_in_frame(cur_gps, goal_gps, orientation, matrix):
    latitude_length=111086.2 
    longitude_length=81978.2 
    delta_lat = goal_gps[0] - cur_gps[0]
    delta_lon = cur_gps[1] - goal_gps[1]
    north_m = delta_lat * latitude_length
    west_m = delta_lon * longitude_length
    desired_heading_x = math.cos(orientation) * west_m + math.sin(orientation) * north_m
    desired_heading_y = -math.sin(orientation) * west_m + math.cos(orientation) * north_m
    
    
    #may be wise to plot this to ensure that they are being added and subtracted correctly based on top being minimum and positive x being right.
    waypoint_position = [matrix.shape[0] + (.05 * desired_heading_y), matrix.shape[1]/2 + .05 * desired_heading_x]

    #if waypoint is within frame: return position
    if waypoint_position[0] >= 0 and waypoint_position[0] <= matrix.shape[0] and waypoint_position[1] >= 0 and waypoint_position[1] <= matrix.shape[1]:
        return True
    return False

#returns the waypoint position. Just an expansion of waypoint_in_frame
def calculate_waypoint_frame_position(cur_gps, goal_gps, orientation, matrix):
    latitude_length=111086.2 
    longitude_length=81978.2 
    delta_lat = goal_gps[0] - cur_gps[0]
    delta_lon = cur_gps[1] - goal_gps[1]
    north_m = delta_lat * latitude_length
    west_m = delta_lon * longitude_length
    desired_heading_x = math.cos(orientation) * west_m + math.sin(orientation) * north_m
    desired_heading_y = -math.sin(orientation) * west_m + math.cos(orientation) * north_m
    
    
    #may be wise to plot this to ensure that they are being added and subtracted correctly based on top being minimum
    # PROBABLE SOURCE OF ERROR: + vs. - for finding x and y positions in frame. 
    # For a waypoint that is within frame in reality, does adding the y component and matrix.shape[0] result in a sum that is within frame in code?

    waypoint_position = (matrix.shape[0] + (.05 * desired_heading_y), matrix.shape[1]/2 + .05 * desired_heading_x)

    return waypoint_position



    #bfs_with_cost will call this function, and it will check if the found space is driveable. [-1, -1] and a high-cost position will both return false.
    #may also need a different way to set a new goal if the current one is within frame, but not driveable.


def calculate_cost(real_rob_pose, orientation ,desire_heading, start, current, rows, cols, matrix, using_angle): 
    edge_penalty_factor=.025
    distance_weight=.5
    min_distance=2
    start_penalty_factor=100
    angle_pen_weight = 0.5
    obs_factor = 1.5

    # edge_penalty_factor=0
    # distance_weight=0
    # min_distance=2
    # start_penalty_factor=100
    # angle_pen_weight = 1000
    # obs_factor = 1
    
    angle_pen = 0
    if using_angle:
        angle_pen = get_angle_to_goal_pentaly(current, real_rob_pose, orientation, desire_heading)

    y_start, x_start = start
    y_current, x_current = current
    
    # Euclidean distance
    # euclidean_distance = (math.sqrt((x_current - x_start)**2 + (y_current - y_start)**2))
    euclidean_distance = (3 * ((y_current - y_start) ** 2))

    # Ensure the distance is not zero when current == start
    if euclidean_distance == 0:
        euclidean_distance = min_distance  # Avoid zero distance

    # Apply weight to the distance
    weighted_distance = distance_weight * euclidean_distance

    # Pull from inflation layer 
    min_distance_to_obstacle = matrix[y_current][x_current]
    
    # Edge penalty
    edge_penalty = min(x_current, cols - x_current - 1, y_current, rows - y_current - 1)
    edge_penalty = max(0, edge_penalty)  # Ensure non-negative
    
    close_pen = 0
    # Penalize if the current point is too close to the start
    if euclidean_distance <= min_distance:
        close_pen += start_penalty_factor  # Add penalty to move away from the start

    # Final cost
   
    cost = close_pen + 4*(1/(weighted_distance+1)) + obs_factor * min_distance_to_obstacle + edge_penalty_factor * (1 / (edge_penalty + 1)) + angle_pen * angle_pen_weight
   
    return cost


# BFS Function

def bfs_with_cost(robot_pose, matrix, start_bfs, directions, current_gps=0, goal_gps=0, robot_orientation=0, using_angle=False):
    # Calculate cost for this cell
    current_gps = (42.668086, -83.218446) # TODO get this from sensors
    goal_gps = (42.6679277, -83.2193276) # TODO get this from publisher
    robot_orientation = math.radians(270) #TODO get this from sensors

    # upper floor test  
    current_gps = (42.29464338650299,-83.70948627128159) # TODO get this from sensors
    goal_gps = (42.29464338650299,-83.70939437630648) # TODO get this from publisher
    robot_orientation = math.radians(69) #TODO get this from sensors
    using_angle = False

    rows, cols = matrix.shape
    visited = set()
    queue = deque([start_bfs])
    visited.add(start_bfs)

    min_cell_cost = float('inf')
    best_cell = None
    visualize_cost_map(matrix)
    goal_cost_matrx = np.zeros_like(matrix, dtype=np.float64) + 100.0

    where_visted = np.zeros_like(matrix)
    num_visted = 0
    # visualize_cost_map(goal_cost_matrx)

    #if the waypoint is within frame, it is automatically the goal.
    if waypoint_in_frame(current_gps, goal_gps, robot_orientation, matrix):
        waypoint = calculate_waypoint_frame_position(current_gps, goal_gps, robot_orientation, matrix)
        
        return waypoint, calculate_cost(robot_pose, robot_orientation, d_heading, start_bfs, waypoint, rows, cols, matrix, using_angle)

    while queue:
        num_visted += 1
        y, x = queue.pop() # pop for dfs pop left for bfs

        d_heading = 0
        if using_angle:
            d_heading = find_desired_heading(current_gps, goal_gps, robot_orientation)
        
        cost = calculate_cost(robot_pose, robot_orientation, d_heading, start_bfs, (y, x), rows, cols, matrix, using_angle)
        goal_cost_matrx[y][x] = cost
        where_visted[y][x] = 1
        if cost < min_cell_cost: 
            min_cell_cost = cost
            best_cell = (y, x)
        # Explore neighbors 
        for dy, dx in directions:
            ny, nx = y + dy, x + dx
            notNearTop = 2 <= ny < rows 
            notOutOfBounds = 0 <= ny < rows and 0 <= nx < cols
            vaild_pixel = matrix[ny, nx] < 100 and matrix[ny, nx] > -1 
            if notNearTop and notOutOfBounds and vaild_pixel and (ny, nx) not in visited:
                # Check all 8 surrounding pixels
                if all(0 <= ny + dy < rows and 0 <= nx + dx < cols and -1 < matrix[ny + dy, nx + dx] < 100 
                    for dy in [-1, 0, 1] for dx in [-1, 0, 1] if (dy, dx) != (0, 0)):
                    queue.append((ny, nx))
                    visited.add((ny, nx))
    # visualize_cost_map(where_visted)
    # visualize_cost_map(goal_cost_matrx)

    #if waypoint in frame, goal_cost
    print("BEST CELL", best_cell)
    print("BEST COST", min_cell_cost)

    visualize_matrix_with_goal(goal_cost_matrx,robot_pose, best_cell) # fav print
    print("Number of cells visited: ", num_visted)
    visualize_cost_map(where_visted)
    # max_value = np.max(goal_cost_matrx)
    # min_value = np.min(goal_cost_matrx)

    # print("Maximum value:", max_value)
    # print("Minimum value:", min_value)
    return best_cell, min_cell_cost

# Visualize the cost map
def visualize_cost_map(cost_map):
    plt.figure(figsize=(10, 8))
    plt.imshow(cost_map, cmap='viridis', origin='upper')
    plt.colorbar(label='Cost')
    plt.title("Cost Heatmap")
    plt.xlabel("X-axis (Columns)")
    plt.ylabel("Y-axis (Rows)")
    plt.show()




def visualize_matrix_with_goal(matrix, start, goal):
    print(" GOAL ", goal)
    print(" START ", start)
    plt.figure(figsize=(8, 8))
    plt.imshow(matrix, cmap="viridis") 
    plt.colorbar(label='Cost')

    # Mark start and goal points
    plt.scatter(start[1], start[0], color="blue", label="Start", s=100)
    plt.scatter(goal[1], goal[0], color="red", label="Goal", s=100)

    # Add grid for clarity
    plt.grid(color="black", linestyle="--", linewidth=0.5)

    # Set tick positions to avoid crowding
    plt.xticks(np.arange(0, matrix.shape[1], step=max(1, matrix.shape[1] // 10)))
    plt.yticks(np.arange(0, matrix.shape[0], step=max(1, matrix.shape[0] // 10)))

    plt.legend()
    plt.title("Matrix Visualization with Start and Goal")
    plt.show()
