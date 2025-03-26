# Navigation Infrastructure
## Usage
To use nav_infrastructure, clone the repository into your `src` directory within your ROS2 workspace: 
```bash
cd <ros2_ws>/src
git clone https://github.com/umigv/nav_infrastructure
```

Navigate back to your workspace and build:
```bash
cd ..
colcon build
```

Now you can run the infrastructure:
```bash
ros2 launch infra_launch infra.launch.py
```

## Parameters
The infrastructure has several parameters that can be set to alter its behavior. They can be found in `infra_launch/config/infra_params.yaml`. 

### Planner server parameters
- `planner_plugin`: class name of the planner plugin to use
- `isolate_path_planner`: can be used to test only the path planning algorithm in isolation
	- When set to true, the `navigate_to_goal` action doesn't call the `follow_path` action and instead publishes poses from the path calculated by the current planner plugin to the navigate_to_goal feedback topic
	- When set to false, functions normally

### Controller server parameters
- `controller_plugin`: class name of the controller plugin to use
- `cmd_vel_topic`: name of the topic to which command velocities produced by the `follow_path` action are published to
	- This topic must be of type `geometry_msgs/Twist`
- `odom_topic`: name of the odometry topic that the controller server will subscribe to
	- This topic must be of type `nav_msgs/Odometry`
- `velocity_update_frequency`: seconds between computing command velocities

## Planner Server
Run the planner server: 
```bash
ros2 launch planner_server planner_server.launch.py
```

The planner server provides the `navigate_to_goal` action, which navigates the robot from the given start coordinate to the given goal coordinate within the given costmap. The action publishes the robot's current distance from the start coordinate to the feedback topic. The action's interface is as follows:
```
# Input
nav_msgs/OccupancyGrid costmap
	# https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html for details on this message type	
Coordinate2D start
	int64 x
	int64 y
Coordinate2D goal
	int64 x
	int64 y
---

# Final output
bool success
---

# Feedback
geometry_msgs/Pose distance_from_start
	# https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html for details on this message type
```

You can call the action from the terminal with this command, inputting your own data for the start, goal, and costmap (the planner_server node must be running for this command to work):
```
ros2 action send_goal /navigate_to_goal infra_interfaces/action/NavigateToGoal "{costmap: {header: {frame_id: 'map'}, info: {width: 2, height: 2, resolution: 1.0, origin: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, data: [0, 0, 0, 0]}, start: {x: 0, y: 0}, goal: {x: 0, y: 1}}"
```

You can inspect the `navigate_to_goal` feedback topic with this command:

```
ros2 topic echo /navigate_to_goal/_action/feedback
```

The planner server relies on plugins to determine and implement the actual search algorithm. To write a planner plugin, consult the nav_plugins documentation: https://github.com/umigv/nav_plugins

## Controller Server
Run the controller server:
```bash
ros2 launch controller_server controller_server.launch.py
```

The controller server provides the `follow_path` action, which sends command velocities to the motors (simulated or physical) to navigate the robot along the given path. The action's interface is as follows: 

```
CellCoordinateMsg[] path
float32 resolution # meters/costmap unit, (1, 0) in costmap coords is (resolution, 0) in meters
---

# Final output
bool success
---
```

You can call the `follow_path` action from the terminal with this command, inputting your own data for the path and resolution (the controller_server node must be running for this command to work):

```
ros2 action send_goal /follow_path infra_interfaces/action/FollowPath "{path: [{x: 0, y: 0}, {x: 1, y: 0}], resolution: 1.0}"
```

Similar to the planner server, the controller server relies on plugins to implement the actual local planning algorithm that produces the command velocities, such as pure pursuit. To write a controller plugin, consult the nav_plugins documentation: https://github.com/umigv/nav_plugins