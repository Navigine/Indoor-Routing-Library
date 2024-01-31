#pragma once

#include <navigine/route_graph/utils.h>

namespace navigine::route_graph {

struct LevelPoint
{
    TId level;
    geometry::XYPoint point;
};

inline bool operator==(const LevelPoint& p1, const LevelPoint& p2)
{
    return p1.level == p2.level && p1.point == p2.point;
}

inline std::ostream& operator<<(std::ostream& out, const LevelPoint& p)
{
    out << "LevelPoint[level=" << p.level << ", x=" << p.point.x << ", y=" << p.point.y << "]";
    return out;
}

} // namespace navigine::route_graph
