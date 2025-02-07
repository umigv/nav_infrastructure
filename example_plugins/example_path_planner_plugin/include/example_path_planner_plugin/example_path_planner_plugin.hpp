#ifndef EXAMPLE_PATH_PLANNER_PLUGIN__EXAMPLE_PATH_PLANNER_PLUGIN_HPP_
#define EXAMPLE_PATH_PLANNER_PLUGIN__EXAMPLE_PATH_PLANNER_PLUGIN_HPP_

#include "example_path_planner_plugin/visibility_control.h"
#include "planner_server/path_planner.hpp"
#include "infra_common/costmap.hpp"


class ExamplePathPlannerPlugin : public planner_server::PathPlanner
{
public:
    ExamplePathPlannerPlugin();
    virtual ~ExamplePathPlannerPlugin();

    std::vector<infra_interfaces::msg::Coordinate2D> FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        infra_interfaces::msg::Coordinate2D start,
        infra_interfaces::msg::Coordinate2D goal) override;
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ExamplePathPlannerPlugin, planner_server::PathPlanner)

#endif  // EXAMPLE_PATH_PLANNER_PLUGIN__EXAMPLE_PATH_PLANNER_PLUGIN_HPP_
