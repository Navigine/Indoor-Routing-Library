#pragma once

#include <navigine/route_graph/level_point.h>

namespace navigine::route_graph {

struct RouteEvent
{
    enum class Type {
        TURN_LEFT,
        TURN_RIGHT,
        TRANSITION
    };

    Type type;
    int value;
    double distance;
};

inline bool operator==(const RouteEvent& e1, const RouteEvent& e2)
{
    return e1.type == e2.type && e1.value == e2.value && e1.distance == e2.distance;
}

class RoutePath;
using SharedRoutePath = std::shared_ptr<RoutePath>;

class RoutePath {
public:
    RoutePath(
        const std::vector<LevelPoint>& points,
        const std::vector<RouteEvent>& events);

    double length() const;
    const std::vector<LevelPoint>& points() const;
    const std::vector<RouteEvent>& events() const;

    std::pair<SharedRoutePath, SharedRoutePath> split(double advance) const;

    static SharedRoutePath createSmoothPath(const std::vector<LevelPoint>& points, double smoothRadius);

    bool operator==(const RoutePath& other) const
    {
        return (points_.size() == other.points_.size() &&
            std::equal(points_.begin(), points_.end(), other.points_.begin()) &&
            std::equal(events_.begin(), events_.end(), other.events_.begin()));
    }

    friend std::ostream& operator<<(std::ostream& stream, const RoutePath& routePath);

private:
    const std::vector<LevelPoint> points_;
    const std::vector<RouteEvent> events_;
    const double length_;
};

std::ostream& operator<<(std::ostream& out, const RouteEvent& e);
std::ostream& operator<<(std::ostream& stream, const RoutePath& routePath);

} // namespace navigine::route_graph
