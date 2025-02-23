#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "plugin_base_classes/controller.hpp"
#include "infra_interfaces/action/follow_path.hpp"
#include "infra_interfaces/msg/cell_coordinate_msg.hpp"
#include "infra_common/cell_coordinate.hpp"

namespace controller_server
{

using namespace std::placeholders;
using namespace infra_common;
using namespace infra_interfaces::msg;

using FollowPath = infra_interfaces::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ServerGoalHandle<FollowPath>;

// Call follow_path action with command:
// ros2 action send_goal /follow_path infra_interfaces/action/FollowPath "{path: [{x: 0, y: 0}, {x: 1, y: 1}]}"

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
        _cmd_vel_publisher = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

        std::string controller_plugin = get_parameter("controller_plugin").as_string();
        load_controller_plugin(controller_plugin);

        _velocity_update_frequency = get_parameter("velocity_update_frequency").as_double();

        RCLCPP_INFO(get_logger(), "Odom topic is %s", odom_topic.c_str());
        RCLCPP_INFO(get_logger(), "Command velocity topic is %s", cmd_vel_topic.c_str());
        RCLCPP_INFO(get_logger(), "Velocity update frequency is %f", _velocity_update_frequency);

        reset_starting_pose();
        
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

    void reset_starting_pose()
    {
        geometry_msgs::msg::Point position;
        position.x = 0;
        position.y = 0;
        position.z = 0;
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0;
        orientation.y = 0;
        orientation.z = 0;
        orientation.w = 1;
        _starting_pose.position = position;
        _starting_pose.orientation = orientation;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received odometry message");
        geometry_msgs::msg::Pose absolute_pose = msg->pose.pose;
        update_distance_from_start(pose_difference(absolute_pose, _starting_pose));
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
        std::vector<CellCoordinate> path = convert_path(goal->path);

        _controller->set_path(path);
        

        auto result = std::make_shared<FollowPath::Result>();
        goal_handle->succeed(result);
    }

    static std::vector<CellCoordinate> convert_path(const std::vector<CellCoordinateMsg> &path_msg)
    {
        std::vector<CellCoordinate> path;
        for (CellCoordinateMsg coord_msg : path_msg)
        {
            path.push_back({coord_msg.x, coord_msg.y});
        }
        return path;
    }

    static geometry_msgs::msg::Pose pose_difference(const geometry_msgs::msg::Pose &pose1, 
        const geometry_msgs::msg::Pose &pose2)
    {
        geometry_msgs::msg::Pose diff;
        diff.position.x = pose1.position.x - pose2.position.x;
        diff.position.y = pose1.position.y - pose2.position.y;
        diff.position.z = pose1.position.z - pose2.position.z;
        
        tf2::Quaternion q1, q2;
        tf2::fromMsg(pose1.orientation, q1);
        tf2::fromMsg(pose2.orientation, q2);
        
        tf2::Quaternion q_diff = q2 * q1.inverse();        
        q_diff.normalize();

        diff.orientation = tf2::toMsg(q_diff);
        return diff;
    }

    // If this node's executor ever changes to a MultiThreadedExecutor, 
    // update_distance_from_start and get_distance_from_start will need to be 
    // modified to be thread-safe (e.g. using a mutex)
    // With the current SingleThreadedExecutor (the default), a mutex cannot be
    // used or it will block all other callbacks and deadlock
    void update_distance_from_start(const geometry_msgs::msg::Pose &pose)
    {
        _distance_from_start = pose;
    }

    geometry_msgs::msg::Pose get_distance_from_start()
    {
        return _distance_from_start;
    }

    std::shared_ptr<plugin_base_classes::Controller> _controller;
    rclcpp_action::Server<FollowPath>::SharedPtr _follow_path_server;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
    double _velocity_update_frequency;

    geometry_msgs::msg::Pose _starting_pose; // Recorded at beginning of each FollowPath action execution
    geometry_msgs::msg::Pose _distance_from_start;
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