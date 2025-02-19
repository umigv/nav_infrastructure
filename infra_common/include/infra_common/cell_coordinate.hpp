#ifndef INFRA_COMMON_CELL_COORDINATE_HPP
#define INFRA_COMMON_CELL_COORDINATE_HPP

namespace infra_common
{

struct CellCoordinate
{
    int x;
    int y;

    bool operator==(const CellCoordinate& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const CellCoordinate& other) const
    {
        return !(*this == other);
    }
};

}

#endif