#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "plugin_base_classes/controller.hpp"
#include "infra_interfaces/action/follow_path.hpp"
#include "infra_interfaces/msg/cell_coordinate_msg.hpp"
#include "infra_common/cell_coordinate.hpp"
#include "controller_server/pose_manager.hpp"

namespace controller_server
{

using namespace std::placeholders;
using namespace infra_common;
using namespace infra_interfaces::msg;
using namespace geometry_msgs::msg;

using FollowPath = infra_interfaces::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

// Call follow_path action with command:
// ros2 action send_goal /follow_path infra_interfaces/action/FollowPath "{path: [{x: 0, y: 0}, {x: 1, y: 1}], resolution: 0.5}"

class ControllerServer : public rclcpp::Node
{
public: 

    ControllerServer() 
    : Node("controller_server")
    {
        declare_parameter("controller_plugin", "");
        declare_parameter("cmd_vel_topic", "");
        declare_parameter("odom_topic", "");
        declare_parameter("velocity_update_frequency", 1.0);
       

        std::string odom_topic = get_parameter("odom_topic").as_string();
        _odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(odom_topic, 
            10, 
            std::bind(&ControllerServer::odom_callback, this, _1));

        std::string cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
        _cmd_vel_publisher = create_publisher<Twist>(cmd_vel_topic, 10);

        std::string controller_plugin = get_parameter("controller_plugin").as_string();
        load_controller_plugin(controller_plugin);

        _velocity_update_frequency = get_parameter("velocity_update_frequency").as_double();

        RCLCPP_INFO(get_logger(), "Odom topic is %s", odom_topic.c_str());
        RCLCPP_INFO(get_logger(), "Command velocity topic is %s", cmd_vel_topic.c_str());
        RCLCPP_INFO(get_logger(), "Velocity update frequency is %f", _velocity_update_frequency);

        _follow_path_server = rclcpp_action::create_server<FollowPath>(this,
            "follow_path",
            std::bind(&ControllerServer::handle_goal, this, _1, _2),
            std::bind(&ControllerServer::handle_cancel, this, _1),
            std::bind(&ControllerServer::handle_accepted, this, _1));
    }

private:

    void load_controller_plugin(const std::string &controller_plugin)
    {
        pluginlib::ClassLoader<plugin_base_classes::Controller> controller_loader("plugin_base_classes", "plugin_base_classes::Controller");
        try
        {
            _controller = controller_loader.createSharedInstance(controller_plugin);
            RCLCPP_INFO(get_logger(), "Loaded controller plugin %s successfully", controller_plugin.c_str());
        }
        catch(pluginlib::PluginlibException& ex)
        {
            RCLCPP_ERROR(get_logger(), "The controller plugin failed to load. Error: %s", ex.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Pose abs_pose = msg->pose.pose;
        // RCLCPP_INFO(get_logger(), "Received odometry message, updating absolute position to (%f, %f, %f)", 
        //     abs_pose.position.x, 
        //     abs_pose.position.y, 
        //     abs_pose.position.z);
        _pose_mgr.update_absolute_pose(abs_pose);
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowPath::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        // TODO: Validate action goal
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread(std::bind(&ControllerServer::follow_path, this, _1), goal_handle).detach();
    }

    void follow_path(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Following path");

        auto goal = goal_handle->get_goal();
        double resolution = goal->resolution;
        std::vector<geometry_msgs::msg::Point> path = normalize_path(goal->path, resolution);
        _controller->set_path(path);

        // The poses we are passing to the controller are relative to the costmap, so 
        // set the origin to the absolute pose of the costmap origin
        // _pose_mgr.set_origin(calculate_costmap_origin_abs_pose(path.front()));

        // Using absolute poses, so origin is default pose
        _pose_mgr.set_origin(PoseManager::default_pose());

        auto sleep_duration = std::chrono::milliseconds((int)(1000.0 * _velocity_update_frequency));
        auto result = std::make_shared<FollowPath::Result>();
        result->success = false;
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                RCLCPP_INFO(get_logger(), "Goal was canceled");
                goal_handle->canceled(result);
                return;
            }

            // TODO: implement some sort of timeout in case controller fails to finish following path
            if (_controller->is_finished())
            {
                RCLCPP_INFO(get_logger(), "Finished following path");
                result->success = true;
                break;
            }

            Pose curr_pose = _pose_mgr.get_relative_pose();
            RCLCPP_INFO(get_logger(), "Passing pose to controller plugin: %f, %f, %f", 
                curr_pose.position.x, 
                curr_pose.position.y, 
                curr_pose.position.z);
            Twist cmd_vel = _controller->compute_next_command_velocity(curr_pose, Twist());
            RCLCPP_INFO(get_logger(), "Publishing command velocity: (%f, %f, %f), (%f, %f, %f)", 
                cmd_vel.linear.x, 
                cmd_vel.linear.y, 
                cmd_vel.linear.z,
                cmd_vel.angular.x, 
                cmd_vel.angular.y, 
                cmd_vel.angular.z);
            _cmd_vel_publisher->publish(cmd_vel);

            // TODO: maybe publish feedback? 

            std::this_thread::sleep_for(sleep_duration);
        }
        
        goal_handle->succeed(result);
    }

    // Converts a given path with the given resolution to a path with resolution 1 meter/cell
    std::vector<geometry_msgs::msg::Point> normalize_path(const std::vector<CellCoordinateMsg> &path_msg,
        const double &resolution)
    {
        // We are assuming the robot is currently located at the first cell in the path
        Pose curr_abs_pose = _pose_mgr.get_absolute_pose();
        RCLCPP_INFO(get_logger(), "Normalizing path using current absolute pose: %f, %f, %f", 
            curr_abs_pose.position.x, 
            curr_abs_pose.position.y, 
            curr_abs_pose.position.z);

        // Distance in meters from the costmap origin
        Pose curr_relative_pose = PoseManager::default_pose(); 
        CellCoordinateMsg start_cell = path_msg.front();
        curr_relative_pose.position.x = (double)start_cell.x * resolution;
        curr_relative_pose.position.y = (double)start_cell.y * resolution;

        // To transform path to absolute poses, add absolute pose of costmap origin to every point
        Pose costmap_origin_abs_pose = PoseManager::pose_difference(curr_abs_pose, curr_relative_pose); // change to add

        std::vector<geometry_msgs::msg::Point> path;
        for (CellCoordinateMsg coord_msg : path_msg)
        {
            // Add absolute pose of costmap origin to each point
            geometry_msgs::msg::Point point;
            point.x = coord_msg.x * resolution + costmap_origin_abs_pose.position.x;
            point.y = coord_msg.y * resolution + costmap_origin_abs_pose.position.y;
            point.z = 0;
            path.push_back(point);

            RCLCPP_INFO(get_logger(), "\tPath cell coordinate: %d, %d", coord_msg.x, coord_msg.y);
            RCLCPP_INFO(get_logger(), "\tPath absolute pose: %f, %f, %f", point.x, point.y, point.z);
        }
        return path;
    }

    // Returns the absolute pose of the costmap origin; assumes the robot is currently located
    // at the given cell within the costmap
    // Also assumes resolution of costmap is 1 meter/cell; will need to either pass resolution
    // as part of action goal, or normalize path so resolution is always 1 meter/cell
    Pose calculate_costmap_origin_abs_pose(CellCoordinate curr_cell)
    {
        Pose start_relative_pose = PoseManager::default_pose(); // Relative to costmap origin
        start_relative_pose.position.x = (double)curr_cell.x;
        start_relative_pose.position.y = (double)curr_cell.y;

        Pose start_abs_pose = _pose_mgr.get_absolute_pose();
        Pose costmap_origin_abs_pose = PoseManager::pose_difference(start_abs_pose, start_relative_pose);
        return costmap_origin_abs_pose;
    }

    std::shared_ptr<plugin_base_classes::Controller> _controller;
    rclcpp_action::Server<FollowPath>::SharedPtr _follow_path_server;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
    rclcpp::Publisher<Twist>::SharedPtr _cmd_vel_publisher;
    double _velocity_update_frequency;

    PoseManager _pose_mgr; 
};

} // namespace controller_server

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting controller_server...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_server::ControllerServer>());
    rclcpp::shutdown();
    return 0;
}