# Navigation Infrastructure

## Planner Server
Run planner server with default planner plugin: 
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