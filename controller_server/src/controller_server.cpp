#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"

#include "plugin_base_classes/controller.hpp"

namespace controller_server
{

class ControllerServer : public rclcpp::Node
{
public: 
    ControllerServer() 
    : Node("controller_server")
    {
        // Load plugin parameter here
        std::string controller_plugin = "";
        load_controller_plugin(controller_plugin);
    }

private:
    void load_controller_plugin(const std::string &controller_plugin)
    {
        pluginlib::ClassLoader<plugin_base_classes::Controller> controller_loader("plugin_base_classes", "plugin_base_classes::Controller");
        try
        {
            _controller = controller_loader.createSharedInstance(controller_plugin);
            RCLCPP_INFO(get_logger(), "Loaded controller plugin successfully");
        }
        catch(pluginlib::PluginlibException& ex)
        {
            RCLCPP_ERROR(get_logger(), "The controller plugin failed to load. Error: %s", ex.what());
        }
    }

    std::shared_ptr<plugin_base_classes::Controller> _controller;
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