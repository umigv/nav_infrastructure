
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

def calculate_angle_to_goal(robot_pose, goal_pose):
    """
    Assuming robot is facing positive x direction 
    Returns angle in radians of the goal pose relative to the robot's current position.
    """
    robot_x, robot_y = robot_pose
    goal_x, goal_y = goal_pose
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    heading = math.atan2(dy, dx)
    return heading

def get_angle_to_goal_pentaly(canidate_node, real_robot_pos, robot_compass_heading, desired_heading_global):
    y,x = canidate_node
    outside_point_y, outside_point_x = real_robot_pos
    dx = x - outside_point_x
    dy = y - outside_point_y
    cell_dir_local = calculate_angle_to_goal(real_robot_pos, canidate_node)

    global_cell_dir = robot_compass_heading + cell_dir_local + math.pi * 0.5

    heading_error = abs(get_angle_difference(desired_heading_global, global_cell_dir))
    heading_error_deg = math.degrees(heading_error)
    return heading_error_deg

#checks if the waypoint is in the CV frame. basically just the find_desired_heading, but with some extra math
def waypoint_in_frame(cur_gps, goal_gps, robot_compass_heading, matrix, robot_pose_in_costmap):
    waypoint_position = calculate_waypoint_frame_position(
        cur_gps=cur_gps, 
        goal_gps=goal_gps,
        robot_compass_heading=robot_compass_heading,
        robot_pose_in_costmap=robot_pose_in_costmap
    )
    i, j = waypoint_position
    if (0 <= i < matrix.shape[0] and 0 <= j < matrix.shape[1]):
        return True
    return False

def calculate_waypoint_frame_position(cur_gps, goal_gps, robot_compass_heading, robot_pose_in_costmap):
    latitude_length = 111086.2  
    longitude_length = 81978.2  
    
    delta_lat = goal_gps.lat - cur_gps.lat
    delta_lon = goal_gps.lon - cur_gps.lon
    
    north_m = delta_lat * latitude_length
    east_m = delta_lon * longitude_length
    
    # Transform to robot body frame (forward = x, left = y)
    # Assuming robot_compass_heading is in radians, and 0 radians is facing north
    forward_m = math.sin(robot_compass_heading) * east_m + math.cos(robot_compass_heading) * north_m
    left_m = -math.cos(robot_compass_heading) * east_m + math.sin(robot_compass_heading) * north_m

    resolution = 0.05  # meters/cell
    robot_i, robot_j = robot_pose_in_costmap  # Robot's position in costmap indices
    
    # Convert to costmap coordinates (0,0 at bottom right):
    # - Forward (+) increases i (upward/north)
    # - Left (+) increases j (leftward/west)
    waypoint_i = int(round(robot_i + (forward_m / resolution)))
    waypoint_j = int(round(robot_j + (left_m / resolution)))
    
    return (waypoint_i, waypoint_j)


    #bfs_with_cost will call this function, and it will check if the found space is driveable. [-1, -1] and a high-cost position will both return false.
    #may also need a different way to set a new goal if the current one is within frame, but not driveable.


def calculate_cost(real_rob_pose, robot_compass_heading ,desire_heading, start, goal_candidate, rows, cols, matrix, using_angle): 
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
    # print("using angle", using_angle)
    if using_angle:
        angle_pen = get_angle_to_goal_pentaly(goal_candidate, real_rob_pose, robot_compass_heading, desire_heading)

    x_start, y_start = start
    x_goal, y_goal = goal_candidate
    
    # Euclidean distance
    euclidean_distance2 = (math.sqrt((x_goal - x_start)**2 + (y_goal - y_start)**2))
    euclidean_distance = (3 * ((x_goal - x_start) ** 2))

    # Ensure the distance is not zero when current == start
    if euclidean_distance == 0:
        euclidean_distance = min_distance  # Avoid zero distance

    # Apply weight to the distance
    weighted_distance = distance_weight * euclidean_distance

    # Pull from inflation layer 
    # print("trying cell", (x_current, y_current), "cost", matrix[y_current][x_current])
    min_distance_to_obstacle = matrix[y_goal][x_goal]
    # print("matrix fin")
    
    # Edge penalty
    edge_penalty = min(x_goal, cols - x_goal - 1, y_goal, rows - y_goal - 1)
    edge_penalty = max(0, edge_penalty)  # Ensure non-negative
    
    close_pen = 0
    # Penalize if the current point is too close to the start
    if euclidean_distance2 <= min_distance:
        close_pen += start_penalty_factor  # Add penalty to move away from the start

    # Final cost
   
    cost = close_pen + 20*(1/(weighted_distance+1)) + obs_factor * min_distance_to_obstacle + edge_penalty_factor * (1 / (edge_penalty + 1)) + angle_pen * angle_pen_weight
   
    return cost


# BFS Function

def bfs_with_cost(robot_pose, 
                  matrix, 
                  start_bfs, 
                  directions, 
                  current_gps, 
                  goal_gps, 
                  robot_orientation,
                  robot_compass_heading, 
                  using_angle=False):
    """
    Returns the optimal goal cell, its cost, and whether the goal cell is a waypoint. 
    """
    rows, cols = matrix.shape
    visited = set()
    queue = deque([start_bfs])
    visited.add(start_bfs)

    min_cell_cost = float('inf')
    best_cell = None
    visualize_cost_map(matrix)
    goal_cost_matrx = np.zeros_like(matrix, dtype=np.float64) + 100.0

    where_visted = np.zeros_like(matrix)
    print("START BFS")
    print(f"Starting BFS cell: {start_bfs}")
    where_visted[start_bfs[1]][start_bfs[0]] = 10
    print("START BFS2")

    num_visted = 0
    visualize_cost_map(goal_cost_matrx)

    #if the waypoint is within frame, it is automatically the goal.
    if waypoint_in_frame(current_gps, goal_gps, robot_compass_heading, matrix, robot_pose):
        waypoint = calculate_waypoint_frame_position(current_gps, goal_gps, robot_orientation, robot_pose)
        cost = calculate_cost(real_rob_pose=robot_pose,
                              robot_compass_heading=robot_orientation,
                              desire_heading=d_heading,
                              start=start_bfs,
                              goal_candidate=waypoint,
                              rows=rows,
                              cols=cols,
                              matrix=matrix,
                              using_angle=using_angle)
        return waypoint, cost, True

    while queue:
        # print("queue ")
        num_visted += 1
        x,y = queue.pop() # pop for dfs pop left for bfs

        d_heading = 0
        if using_angle:
            waypoint = calculate_waypoint_frame_position(current_gps, goal_gps, robot_orientation, matrix, (y,x))
            d_heading = calculate_angle_to_goal(robot_pose, waypoint)
        # print("tring cell", (x,y))
        # print("x,y,rows,cols", x,y,rows,cols)
        # print("start bfs", start_bfs)
        cost = calculate_cost(robot_pose, robot_compass_heading, d_heading, start_bfs, (x,y), rows, cols, matrix, using_angle)
        goal_cost_matrx[y][x] = cost
        where_visted[y][x] = 1
        if cost < min_cell_cost: 
            min_cell_cost = cost
            best_cell = (x,y)
        # Explore neighbors 
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            notNearTop = 2 <= nx < cols 
            notOutOfBounds = 0 <= ny < rows and 0 <= nx < cols
            if not notOutOfBounds:
                continue
            # print("trying vaild cell", (nx, ny))
            vaild_pixel = matrix[ny, nx] < 75 and matrix[ny, nx] > -1 
            # print("notNearTop", notNearTop)
            # print("notOutOfBounds", notOutOfBounds)
            # print("vaield_pixel", vaild_pixel)
            # print("x y vistred", (nx, ny) not in visited)
            # print("nx,ny", nx, ny)
            # print("matrix[nx, ny]", matrix[ny, nx])
            if notNearTop and notOutOfBounds and vaild_pixel and (nx, ny) not in visited:
                # Check all 8 surrounding pixels
                # if all(0 <= ny + dy < rows and 0 <= nx + dx < cols and -1 < matrix[ny + dy, nx + dx] < 2000000 
                #     for dy in [-1, 0, 1] for dx in [-1, 0, 1] if (dx, dy) != (0, 0)):
                    queue.append((nx, ny))
                    visited.add((nx, ny))
    
    visualize_cost_map(goal_cost_matrx)

    #if waypoint in frame, goal_cost
    print("BEST CELL", best_cell)
    print("BEST COST", min_cell_cost)
    visualize_cost_map(where_visted)
    visualize_matrix_with_goal(goal_cost_matrx,robot_pose, best_cell) # fav print
    # print("Number of cells visited: ", num_visted)
    visualize_cost_map(where_visted)
    # max_value = np.max(goal_cost_matrx)
    # min_value = np.min(goal_cost_matrx)

    # print("Maximum value:", max_value)
    # print("Minimum value:", min_value)
    return best_cell, min_cell_cost, False

# Visualize the cost map
def visualize_cost_map(cost_map):
    plt.figure(figsize=(10, 8))
    plt.imshow(cost_map, cmap='viridis', origin='lower')
    plt.colorbar(label='Cost')
    plt.title("Cost Heatmap")
    plt.xlabel("X-axis (Columns)")
    plt.ylabel("Y-axis (Rows)")
    plt.show()




def visualize_matrix_with_goal(matrix, start, goal):
    print(" GOAL ", goal)
    print(" START ", start)
    plt.figure(figsize=(8, 8))
    plt.imshow(matrix, cmap="viridis",  origin='lower') 
    plt.colorbar(label='Cost')

    # Mark start and goal points
    plt.scatter(start[0], start[1], color="blue", label="Start", s=100)
    plt.scatter(goal[0], goal[1], color="red", label="Goal", s=100)

    # Add grid for clarity
    plt.grid(color="black", linestyle="--", linewidth=0.5)

    # Set tick positions to avoid crowding
    plt.xticks(np.arange(0, matrix.shape[1], step=max(1, matrix.shape[1] // 10)))
    plt.yticks(np.arange(0, matrix.shape[0], step=max(1, matrix.shape[0] // 10)))

    plt.legend()
    plt.title("Matrix Visualization with Start and Goal")
    plt.show()
