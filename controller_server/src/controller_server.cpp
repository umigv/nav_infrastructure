#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plugin_base_classes/controller.hpp"
#include "infra_interfaces/action/follow_path.hpp"
#include "infra_interfaces/msg/cell_coordinate_msg.hpp"
#include "infra_common/cell_coordinate.hpp"

namespace controller_server
{

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

        _cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
        RCLCPP_INFO(get_logger(), "Command velocity topic is %s", _cmd_vel_topic.c_str());
        std::string controller_plugin = get_parameter("controller_plugin").as_string();
        load_controller_plugin(controller_plugin);

        using namespace std::placeholders;
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

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowPath::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        // TODO: Validate action goal
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
        std::thread(std::bind(&ControllerServer::follow_path, this, std::placeholders::_1), goal_handle).detach();
    }

    void follow_path(const std::shared_ptr<GoalHandleFollowPath> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Following path");
        auto result = std::make_shared<FollowPath::Result>();
        goal_handle->succeed(result);
    }

    std::shared_ptr<plugin_base_classes::Controller> _controller;
    std::string _cmd_vel_topic;
    rclcpp_action::Server<FollowPath>::SharedPtr _follow_path_server;
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