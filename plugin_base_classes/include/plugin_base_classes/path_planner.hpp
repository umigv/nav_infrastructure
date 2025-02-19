#ifndef PLANNER_SERVER_PATH_PLANNER_HPP
#define PLANNER_SERVER_PATH_PLANNER_HPP

#include <vector>
#include <functional>

#include "infra_common/costmap.hpp"
#include "infra_common/cell_coordinate.hpp"


namespace plugin_base_classes
{

class PathPlanner
{
public:
    // Plugins should implement this function
    // Returns a vector of coordinates representing a path from start to goal; path 
    // should include goal but not start
    // If no path is found, an empty vector should be returned
    // drivable is given the cost value of a costmap cell and returns whether that
    // cell is drivable
    virtual std::vector<infra_common::CellCoordinate> FindPath(infra_common::Costmap costmap, 
        std::function<bool(int)> drivable,
        infra_common::CellCoordinate start,
        infra_common::CellCoordinate goal) = 0;
    virtual ~PathPlanner() {}

protected:
    // pluginlib requires default constructor and destructor
    PathPlanner() {}
};

}


#endif