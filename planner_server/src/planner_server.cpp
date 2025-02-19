#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "infra_interfaces/action/navigate_to_goal.hpp"
#include "infra_interfaces/msg/cell_coordinate_msg.hpp"
#include "plugin_base_classes/path_planner.hpp"
#include "infra_common/cell_coordinate.hpp"


namespace planner_server
{

using namespace infra_common;
using namespace infra_interfaces::msg;

using NavigateToGoal = infra_interfaces::action::NavigateToGoal;
using GoalHandleNavigateToGoal = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

class PlannerServer : public rclcpp::Node
{
public:
    PlannerServer() 
    : Node("planner_server")
    {
        declare_parameter("planner_plugin", "");
        declare_parameter("odom_topic", "");

        _odom_topic = get_parameter("odom_topic").as_string();
        RCLCPP_INFO(get_logger(), "Odom topic is %s", _odom_topic.c_str());
        std::string planner_plugin = get_parameter("planner_plugin").as_string();

        load_planner_plugin(planner_plugin);

        using namespace std::placeholders;
        _navigate_server = rclcpp_action::create_server<NavigateToGoal>(this,
            "navigate_to_goal",
            std::bind(&PlannerServer::handle_goal, this, _1, _2),
            std::bind(&PlannerServer::handle_cancel, this, _1),
            std::bind(&PlannerServer::handle_accepted, this, _1));
    }

private:

    void load_planner_plugin(std::string planner_plugin)
    {
        pluginlib::ClassLoader<plugin_base_classes::PathPlanner> planner_loader("plugin_base_classes", "plugin_base_classes::PathPlanner");
        try
        {
            _planner = planner_loader.createSharedInstance(planner_plugin);
            RCLCPP_INFO(get_logger(), "Loaded planner plugin successfully");
        }
        catch(pluginlib::PluginlibException& ex)
        {
            RCLCPP_ERROR(get_logger(), "The planner plugin failed to load. Error: %s", ex.what());
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToGoal::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;

        // Validate the action goal
        try
        {
            Costmap costmap(goal->costmap);
            CellCoordinateMsg start = goal->start;
            CellCoordinateMsg goal_coord = goal->goal;
            if (!costmap.InBounds(start.x, start.y) || !costmap.InBounds(goal_coord.x, goal_coord.y))
            {
                RCLCPP_ERROR(this->get_logger(), "Start or goal coordinate is out of bounds");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create costmap from goal: %s", e.what());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread(std::bind(&PlannerServer::navigate, this, std::placeholders::_1), goal_handle).detach();
    }

    void navigate(const std::shared_ptr<GoalHandleNavigateToGoal> goal_handle)
    {
        auto action_goal = goal_handle->get_goal();
        Costmap costmap(action_goal->costmap);
        CellCoordinateMsg startMsg = action_goal->start;
        CellCoordinateMsg goalMsg = action_goal->goal;
        CellCoordinate start = {startMsg.x, startMsg.y};
        CellCoordinate goal = {goalMsg.x, goalMsg.y};
        RCLCPP_INFO(get_logger(), "Navigating from (%d, %d) to (%d, %d)", start.x, start.y, goal.x, goal.y);

        auto drivable = [](int cost) { return cost == 0; };
        // Need to include start in this path
        std::vector<CellCoordinate> path = _planner->FindPath(costmap, 
            drivable,
            start,
            goal);
        RCLCPP_INFO(this->get_logger(), "Found path with length %ld", path.size());
        // Call local planner plugin here

        // TODO: publish current position to feedback topic and detect when robot reaches goal
        // Basically just need to figure out where to get the robot's current position (get from ZED odometry)
        // https://www.stereolabs.com/docs/ros2/060_positional-tracking#position-info-subscribing-in-c
        // Use "odom" topic, which is of type nav_msgs/Odometry

        // CV determines costmap resolution

        // For now, simulate robot movement by publishing feedback every so often
        auto feedback = std::make_shared<NavigateToGoal::Feedback>();
        auto sleep_duration = std::chrono::milliseconds(500);

        for (CellCoordinate coord : path)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = double(coord.x);
            pose.position.y = double(coord.y);
            feedback->distance_from_start = pose;
            RCLCPP_INFO(get_logger(), "Publishing feedback pose (%f, %f)", pose.position.x, pose.position.y);
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(sleep_duration);
        }

        auto result = std::make_shared<NavigateToGoal::Result>();
        if (path.back() != goal)
        {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(get_logger(), "Navigation failed");
        } 
        else 
        {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Navigation succeeded");
        }
    }

    std::shared_ptr<plugin_base_classes::PathPlanner> _planner;
    rclcpp_action::Server<NavigateToGoal>::SharedPtr _navigate_server;
    std::string _odom_topic;
};  

} // namespace planner_server

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting planner_server...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planner_server::PlannerServer>());
    rclcpp::shutdown();
    return 0;
}