#include <queue>
#include <algorithm>

#include "example_path_planner_plugin/example_path_planner_plugin.hpp"

#include "rclcpp/rclcpp.hpp"

using Coordinate2D = infra_interfaces::msg::Coordinate2D;

ExamplePathPlannerPlugin::ExamplePathPlannerPlugin()
{
}

ExamplePathPlannerPlugin::~ExamplePathPlannerPlugin()
{
}

std::vector<Coordinate2D> ExamplePathPlannerPlugin::FindPath(infra_common::Costmap costmap, 
    std::function<bool(int)> drivable,
    Coordinate2D start,
    Coordinate2D goal)
{
    RCLCPP_INFO(rclcpp::get_logger("ExamplePathPlannerPlugin"), "ExamplePathPlannerPlugin planning path using BFS");
    const int width = costmap.GetWidth();
    const int height = costmap.GetHeight();

    Coordinate2D invalid;
    invalid.x = -1;
    invalid.y = -1;
    std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
    std::vector<std::vector<Coordinate2D>> parent(width, std::vector<Coordinate2D>(height, invalid));
    std::queue<Coordinate2D> queue;

    // Movement directions (right, left, up, down)
    Coordinate2D right, left, up, down;
    right.x = 1;
    right.y = 0;
    left.x = -1;
    left.y = 0;
    up.x = 0;
    up.y = 1;
    down.x = 0;
    down.y = -1;

    const std::vector<Coordinate2D> directions = {right, left, up, down};

    // Initialize BFS
    queue.push(start);
    visited[start.x][start.y] = true;
    parent[start.x][start.y] = invalid;

    bool found = false;

    while (!queue.empty()) {
        Coordinate2D current = queue.front();
        queue.pop();

        // Early exit if goal found
        if (current.x == goal.x && current.y == goal.y) {
            found = true;
            break;
        }

        // Explore neighbors
        for (const auto& dir : directions) {
            const int nx = current.x + dir.x;
            const int ny = current.y + dir.y;

            // Check if neighbor is valid and unvisited
            if (costmap.InBounds(nx, ny) && !visited[nx][ny]) {
                const int cost = costmap.GetCost(nx, ny);
                if (drivable(cost)) {
                    visited[nx][ny] = true;
                    parent[nx][ny] = current;
                    Coordinate2D next_coord;
                    next_coord.x = nx;
                    next_coord.y = ny;
                    queue.push(next_coord);
                }
            }
        }
    }

    if (!found) {
        return {};
    }

    // Reconstruct path from goal to start
    std::vector<Coordinate2D> path;
    Coordinate2D current = goal;
    while (!(current.x == -1 && current.y == -1)) {
        path.push_back(current);
        current = parent[current.x][current.y];
    }

    // Reverse to get start-to-goal order
    std::reverse(path.begin(), path.end());

    return path;
}