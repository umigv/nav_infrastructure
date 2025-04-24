#include "infra_common/costmap.hpp"

using namespace infra_common;

Costmap::Costmap(const nav_msgs::msg::OccupancyGrid occupancy_grid_in)
    : _occupancy_grid(occupancy_grid_in)
    { 
        if (_occupancy_grid.data.size() != _occupancy_grid.info.width * _occupancy_grid.info.height)
        {
            throw std::invalid_argument("Costmap::Costmap: data size does not match width * height");
        }
    }

int Costmap::GetCost(int x, int y) const
{
    if (!InBounds(x, y))
    {
        throw std::out_of_range("Costmap::GetCost: out of range");
    }
    return _occupancy_grid.data[y * GetWidth() + x];
}

bool Costmap::InBounds(int x, int y) const
{
    
    return x >= 0 && x < GetWidth() && y >= 0 && y < GetHeight();
}

int Costmap::GetWidth() const
{   
    return _occupancy_grid.info.width;
}

int Costmap::GetHeight() const
{
    return _occupancy_grid.info.height;
}