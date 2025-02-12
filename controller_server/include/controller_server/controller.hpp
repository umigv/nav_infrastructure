#ifndef CONTROLLER_SERVER_LOCAL_PLANNER_HPP
#define CONTROLLER_SERVER_LOCAL_PLANNER_HPP

#include <vector>

#include "infra_interfaces/msg/coordinate2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace controller_server
{

class Controller
{
public:
    virtual void set_path(const std::vector<infra_interfaces::msg::Coordinate2D> &path) = 0;

    virtual geometry_msgs::msg::Twist compute_next_command_velocity(const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::Twist &current_velocity) = 0;

    virtual ~Controller() {}

protected:
    Controller() {}
};


} // namespace controller_server


#endif