#ifndef PLUGIN_BASE_CLASSES_CONTROLLER_HPP
#define PLUGIN_BASE_CLASSES_CONTROLLER_HPP

#include <vector>

#include "infra_common/cell_coordinate.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace plugin_base_classes
{

class Controller
{
public:
    virtual void set_path(const std::vector<geometry_msgs::msg::Point> &path) = 0;

    virtual geometry_msgs::msg::Twist compute_next_command_velocity(const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::Twist &current_velocity) = 0;

    virtual bool is_finished() const = 0;

    virtual ~Controller() {}

protected:
    Controller() {}
};


} // namespace controller_server


#endif