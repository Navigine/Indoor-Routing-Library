#include <string>
#include <vector>

#include <navigine/route_graph/route_path.h>

#include "test_utils.h"

using namespace navigine;
using namespace navigine::route_graph;

namespace {

constexpr TId LEVEL_1 = 1;
constexpr TId LEVEL_2 = 2;

std::vector<LevelPoint> createRoutePoints(
    std::vector<std::tuple<TId, double, double>> values)
{
    std::vector<LevelPoint> points;
    for (auto&& [levelId, x, y] : values) {
        points.push_back(LevelPoint{.level=levelId, .point=geometry::XYPoint{.x=x, .y=y}});
    }
    return points;
}

std::vector<RouteEvent> createRouteEvents(
    std::vector<std::tuple<RouteEvent::Type, int, double>> values)
{
    std::vector<RouteEvent> events;
    for (auto&& [type, value, distance] : values) {
        events.push_back(RouteEvent{.type = type, .value = value, .distance = distance});
    }
    return events;
}

} // namespace

int test_split_case_1()
{
    auto routePath = std::make_shared<RoutePath>(
        createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
        createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TURN_RIGHT, 90, 2}}));

    // std::cout << std::endl << "Full route:" << std::endl;
    // printRoutePath(routePath);

    {
        auto [left, right] = routePath->split(0.5);

        // std::cout<< std::endl << "Left route:" << std::endl;
        // printRoutePath(left);

        // std::cout<< std::endl << "Right route:" << std::endl;
        // printRoutePath(right);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 0.5}}),
            createRouteEvents({}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0.5}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 0.5}, {RouteEvent::Type::TURN_RIGHT, 90, 1.5}}));
        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }
        return 0;
    }

    {
        auto [left, right] = routePath->split(1.5);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 0.5, 1}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({ {LEVEL_1, 0.5, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
            createRouteEvents({ {RouteEvent::Type::TURN_RIGHT, 90, 0.5}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(2.5);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0.5}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TURN_RIGHT, 90, 2}}));
        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({ {LEVEL_1, 1, 0.5}, {LEVEL_1, 1, 0}}),
            createRouteEvents({ }));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }
}

int test_split_case_2()
{
    auto routePath = std::make_shared<RoutePath>(
        createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
        createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TURN_RIGHT, 90, 2}}));

    {
        auto [left, right] = routePath->split(0);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}}),
            createRouteEvents({}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TURN_RIGHT, 90, 2}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(1);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}}),
            createRouteEvents({}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({ {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
            createRouteEvents({ {RouteEvent::Type::TURN_RIGHT, 90, 1}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(2);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({ {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
            createRouteEvents({ }));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(3);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_1, 1, 0}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TURN_RIGHT, 90, 2}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 1, 0}}),
            createRouteEvents({ }));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

}
int test_split_case_3()
{
    auto routePath = std::make_shared<RoutePath>(
        createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}, {LEVEL_2, 1, 0}, {LEVEL_2, 0, 0}}),
        createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TRANSITION, 2, 2}, {RouteEvent::Type::TURN_RIGHT, 90, 3}}));

    {
        auto [left, right] = routePath->split(0.5);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 0.5}}),
            createRouteEvents({}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0.5}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}, {LEVEL_2, 1, 0}, {LEVEL_2, 0, 0}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 0.5}, {RouteEvent::Type::TRANSITION, 2, 1.5}, {RouteEvent::Type::TURN_RIGHT, 90, 2.5}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(1.5);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 0.5, 1}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0.5, 1}, {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}, {LEVEL_2, 1, 0}, {LEVEL_2, 0, 0}}),
            createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 0.5}, {RouteEvent::Type::TURN_RIGHT, 90, 1.5}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(2);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TRANSITION, 2, 2}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({ {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}, {LEVEL_2, 1, 0}, {LEVEL_2, 0, 0}}),
            createRouteEvents({ {RouteEvent::Type::TRANSITION, 2, 0}, {RouteEvent::Type::TURN_RIGHT, 90, 1}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(2.5);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}, {LEVEL_2, 1, 0.5}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TRANSITION, 2, 2}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_2, 1, 0.5}, {LEVEL_2, 1, 0}, {LEVEL_2, 0, 0}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 0.5}}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }

    {
        auto [left, right] = routePath->split(3.5);

        auto res1 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_1, 0, 0}, {LEVEL_1, 0, 1}, {LEVEL_1, 1, 1}, {LEVEL_2, 1, 1}, {LEVEL_2, 1, 0}, {LEVEL_2, 0.5, 0}}),
            createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 1}, {RouteEvent::Type::TRANSITION, 2, 2}, {RouteEvent::Type::TURN_RIGHT, 90, 3}}));

        if (!test_utils::testEqualValues(*left, *res1)) {
            return -1;
        }

        auto res2 = std::make_shared<RoutePath>(
            createRoutePoints({{LEVEL_2, 0.5, 0}, {LEVEL_2, 0, 0}}),
            createRouteEvents({}));

        if (!test_utils::testEqualValues(*right, *res2)) {
            return -1;
        }

        return 0;
    }
}
