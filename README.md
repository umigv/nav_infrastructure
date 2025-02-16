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

### Creating a path planner plugin
The planner server dynamically loads a path planner plugin at runtime, making it easy to swap between different path planning algorithms. Here's how to create a path planner plugin:

- For this example, we will create a path planner plugin called `path_planner_plugin`; replace this with whatever you want to call your plugin (make it descriptive based on what algorithm it's implementing)
	- `path_planner_plugin` will also be the name of the ROS2 package the plugin is contained within
- While creating your plugin, you can always use `example_plugins/example_path_planner_plugin`
- Create the path planner plugin package with the following command:
```bash
ros2 pkg create --build-type ament_cmake path_planner_plugin --dependencies rclcpp planner_server pluginlib infra_common infra_interfaces --library-name path_planner_plugin
```
- Open `include/path_planner_plugin/path_planner_plugin.hpp`, and add the following includes at the top:
```cpp
#include "planner_server/path_planner.hpp"
#include "infra_common/costmap.hpp"
```
- Make your plugin class derive from `planner_server::PathPlanner`:
```cpp
class PathPlannerPlugin : public planner_server::PathPlanner
```
- Run `colcon build` to make sure this builds
- Open your source file: `src/path_planner_plugin.cpp`
- Using `example_path_planner_plugin.cpp` and `example_path_planner_plugin.hpp` in the `example_path_planner_plugin` package (in the `example_plugins` folder within this repository) as reference, implement the necessary virtual functions from the `PathPlanner` abstract class to add your path planning logic
- Create a `plugins.xml` file in the root of your plugin package and enter the following code:
```xml
<library path="path_planner_plugin">
  <class type="PathPlannerPlugin" base_class_type="planner_server::PathPlanner">
    <description>Description of your plugin here.</description>
  </class>
</library>
```
- Open `CMakeLists.txt`, and add the following line after the line reading `find_package(pluginlib REQUIRED)`:
```cmake
pluginlib_export_plugin_description_file(planner_server plugins.xml)
```
- Once your plugin is building successfully, you'll need to edit the planner server parameters to actually use your plugin
	- Open `nav_infrastructure/planner_server/config/planner_server_params.yaml`
	- Edit the `planner_plugin` to contain the complete name of your plugin class (this should match the name you put in your `plugins.xml` file):
	```yaml
	planner_plugin: "ExamplePathPlannerPlugin"
	```
- Now launch the planner server and it should use your plugin to create the path
