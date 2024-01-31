#pragma once

#include <cstdint>

#include <navigine/geometry/geometry.h>

namespace navigine::route_graph {

using TId = uint64_t;

const double EPSILON = 1e-10;

template<typename Point>
geometry::XYPoint GetPoint(const Point& point);

template<typename Point>
std::vector<Point> SimplifyPolyline(const std::vector<Point>& points, double radius)
{
    // simplify the polyline using the Douglas-Peuker algorithm
    std::vector<bool> keepPoint(points.size(), true);
    std::vector<std::pair<int,int>> stack;
    stack.push_back(std::make_pair(0, points.size() - 1));

    while (!stack.empty()) {
        const auto [startPos, stopPos] = stack.back();
        stack.pop_back();

        if (stopPos - startPos <= 1) {
            continue;
        }

        double dmax = 0.0;
        int pos = startPos;

        for(int i = startPos + 1; i <= stopPos - 1; ++i) {
            if (keepPoint[i]) {
                const auto line = geometry::XYSegment{
                    GetPoint(points[startPos]),
                    GetPoint(points[stopPos])};
                const auto d = geometry::LinePointDist(line, GetPoint(points[i]));
                if (d > dmax) {
                    dmax = d;
                    pos = i;
                }
            }
        }

        if (dmax >= radius) {
            stack.push_back(std::make_pair(startPos, pos));
            stack.push_back(std::make_pair(pos, stopPos));
        } else {
            for(int i = startPos + 1; i <= stopPos - 1; ++i) {
                keepPoint[i] = false;
            }
        }
    }

    std::vector<Point> points1;
    for(size_t i = 0; i < points.size(); ++i) {
        if (keepPoint[i]) {
            points1.push_back(points[i]);
        }
    }

    return points1;
}

} // namespace navigine::route_graph
