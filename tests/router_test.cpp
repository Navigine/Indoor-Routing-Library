#include <string>
#include <vector>

#include <navigine/route_graph/route_graph.h>
#include <navigine/route_graph/router.h>

#include "test_utils.h"

using namespace navigine;
using namespace navigine::route_graph;

namespace {

constexpr TId LEVEL1 = 1;
constexpr TId LEVEL2 = 2;

bool isEqual(const SharedRoutePath& r1, const SharedRoutePath& r2)
{
    if (!r1 && !r2) {
        return true;
    }
    if (!r1 || !r2) {
        return false;
    }

    return *r1 == *r2;
}

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

int test_router_basic_test_case()
{
    auto routeGraph = std::make_shared<RouteGraph>();
    routeGraph->addLevel(LEVEL1);

    auto stop = LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}};

    auto router = Router(
        stop,
        route_graph::Router::Options{
            .smoothRadius = 0.0,
            .maxProjectionDistance = 5.0,
            .maxAdvance = 2.0,
        },
        [](const SharedRoutePath& path) {
            if (!path) {
                throw std::runtime_error("path should NOT been nullptr");
            }
        },
        [](double , const LevelPoint& ){
            throw std::runtime_error("OnRouteAdvanced should NOT been called");
        });

    try {
        router.updateGraph(routeGraph);
        router.updatePosition(std::nullopt);
    } catch(const std::runtime_error& e) {
        std::cout << e.what();
        return -1;
    }
    return 0;
}

int test_router_basic_test_path1()
{
    auto routeGraph = std::make_shared<RouteGraph>();
    routeGraph->addLevel(LEVEL1);
    routeGraph->addVertex(RouteGraph::Vertex{.id=1,.levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=2,.levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=10}}, .isElevation = false, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=3,.levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=10}}, .isElevation = false, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=4,.levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=0}}, .isElevation = false, .name = ""});

    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=1, .src=1, .dst=2, .weight=1.0});
    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=2, .src=2, .dst=3, .weight=1.0});
    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=3, .src=3, .dst=4, .weight=1.0});

    auto stop = LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=3, .y=-1}};

    SharedRoutePath curPath;
    double curDistance = 0;
    LevelPoint curPoint;
    size_t onRouteChanged = 0;
    size_t onRouteAdvance = 0;

    auto checkTest = [&](
        const std::string& name,
        const SharedRoutePath& expectedPath,
        double expectedDistance,
        const LevelPoint& expectedPoint,
        size_t expectedRouteChanged,
        size_t expectedRouteAdvance)
    {
        if (!isEqual(curPath, expectedPath)) {
            std::cout << "ERROR in " << name << std::endl;
            if (curPath) {
                std::cout << "curPath: " << std::endl << *curPath << std::endl;
            }
            if (expectedPath) {
                std::cout << "expectedPath: " << std::endl << *expectedPath << std::endl;
            }
            throw std::runtime_error("Failed");
        }

        if (curPoint != expectedPoint) {
            std::cout << "ERROR in " << name << std::endl;
            std::cout << "point=" << curPoint << " expected=" << expectedPoint << std::endl;
            throw std::runtime_error("Failed");
        }

        if (curDistance != expectedDistance) {
            std::cout << "ERROR in " << name << std::endl;
            std::cout << "distance=" << curDistance << " expected=" << expectedDistance << std::endl;
            throw std::runtime_error("Failed");
        }

        if (!test_utils::testEqualValues(onRouteChanged, expectedRouteChanged) ||
            !test_utils::testEqualValues(onRouteAdvance, expectedRouteAdvance)) {
            throw std::runtime_error("Failed");
        }
    };

    auto router = Router(
        stop,
        route_graph::Router::Options{
            .smoothRadius = 0.0,
            .maxProjectionDistance = 5.0,
            .maxAdvance = 2.0,
        },
        [&](const SharedRoutePath& path) mutable {
            curPath = path;
            onRouteChanged++;
        },
        [&](double distance, const LevelPoint& point) mutable {
            curDistance = distance;
            curPoint = point;
            onRouteAdvance++;
        });

    router.updateGraph(routeGraph);

    try {
        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-10, .y=-10}});
        checkTest(
            "TestGetPath1.1",
            nullptr,
            0,
            curPoint,
            0,
            0);
        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=-1}});
        checkTest(
            "TestGetPath1.2",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL1, 2, 10}, {LEVEL1, 2, 0}, {LEVEL1, 3, -1}}),
                createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 10}, {RouteEvent::Type::TURN_RIGHT, 90, 12}, {RouteEvent::Type::TURN_LEFT, 45, 22}})),
            0,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}},
            1,
            1);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=1}});
        checkTest(
            "TestGetPath1.3",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL1, 2, 10}, {LEVEL1, 2, 0}, {LEVEL1, 3, -1}}),
                createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 10}, {RouteEvent::Type::TURN_RIGHT, 90, 12}, {RouteEvent::Type::TURN_LEFT, 45, 22}})),
            1,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=1}},
            1,
            2);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=-1}});
        checkTest(
            "TestGetPath1.4",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL1, 2, 10}, {LEVEL1, 2, 0}, {LEVEL1, 3, -1}}),
                createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 10}, {RouteEvent::Type::TURN_RIGHT, 90, 12}, {RouteEvent::Type::TURN_LEFT, 45, 22}})),
            1,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=1}},
            1,
            2);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1.5, .y=2}});
        checkTest(
            "TestGetPath1.5",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL1, 2, 10}, {LEVEL1, 2, 0}, {LEVEL1, 3, -1}}),
                createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 10}, {RouteEvent::Type::TURN_RIGHT, 90, 12}, {RouteEvent::Type::TURN_LEFT, 45, 22}})),
            2,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=2}},
            1,
            3);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1.5, .y=11}});
        checkTest(
            "TestGetPath1.6",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 1.5, 10}, {LEVEL1, 2, 10}, {LEVEL1, 2, 0}, {LEVEL1, 3, -1}}),
                createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 0.5}, {RouteEvent::Type::TURN_LEFT, 45, 10.5}})),
            0,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1.5, .y=10}},
            2,
            4);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=-1}});
        checkTest(
            "TestGetPath1.7",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL1, 2, 10}, {LEVEL1, 2, 0}, {LEVEL1, 3, -1}}),
                createRouteEvents({{RouteEvent::Type::TURN_RIGHT, 90, 10}, {RouteEvent::Type::TURN_RIGHT, 90, 12}, {RouteEvent::Type::TURN_LEFT, 45, 22}})),
            0,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}},
            3,
            5);
    } catch (const std::runtime_error&) {
        return -1;
    }
    return 0;
}

int test_router_basic_test_path2()
{
    auto routeGraph = std::make_shared<RouteGraph>();
    routeGraph->addLevel(LEVEL1);
    routeGraph->addLevel(LEVEL2);

    routeGraph->addVertex(RouteGraph::Vertex{.id=1, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=2, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=10}}, .isElevation = true, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=3, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=10}}, .isElevation = true, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=4, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});

    routeGraph->addVertex(RouteGraph::Vertex{.id=5, .levelPoint=LevelPoint{.level=LEVEL2, .point=geometry::XYPoint{.x=0, .y=10}}, .isElevation = true, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=6, .levelPoint=LevelPoint{.level=LEVEL2, .point=geometry::XYPoint{.x=1, .y=10}}, .isElevation = true, .name = ""});


    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=1, .src=1, .dst=2, .weight=1.0});
    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=2, .src=2, .dst=3, .weight=1.0});
    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=3, .src=3, .dst=4, .weight=1.0});

    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL2, .id=4, .src=5, .dst=6, .weight=0.5});

    routeGraph->addEdge(RouteGraph::Edge{.level=RouteGraph::Edge::ELEVATION_LEVEL_ID, .id=5, .src=2, .dst=5, .weight=0});
    routeGraph->addEdge(RouteGraph::Edge{.level=RouteGraph::Edge::ELEVATION_LEVEL_ID, .id=6, .src=6, .dst=3, .weight=0});

    auto stop = LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=-1}};

    SharedRoutePath curPath;
    double curDistance = 0;
    LevelPoint curPoint;
    size_t onRouteChanged = 0;
    size_t onRouteAdvance = 0;

    auto checkTest = [&](
        const std::string& name,
        const SharedRoutePath& expectedPath,
        double expectedDistance,
        const LevelPoint& expectedPoint,
        size_t expectedRouteChanged,
        size_t expectedRouteAdvance)
    {
        if (!isEqual(curPath, expectedPath)) {
            std::cout << "ERROR in " << name << std::endl;
            if (curPath) {
                std::cout << "curPath: " << std::endl << *curPath << std::endl;
            }
            if (expectedPath) {
                std::cout << "expectedPath: " << std::endl << *expectedPath << std::endl;
            }
            throw std::runtime_error("Failed");
        }

        if (curPoint != expectedPoint) {
            std::cout << "ERROR in " << name << std::endl;
            std::cout << "point=" << curPoint << " expected=" << expectedPoint << std::endl;
            throw std::runtime_error("Failed");
        }

        if (curDistance != expectedDistance) {
            std::cout << "ERROR in " << name << std::endl;
            std::cout << "distance=" << curDistance << " expected=" << expectedDistance << std::endl;
            throw std::runtime_error("Failed");
        }

        if (!test_utils::testEqualValues(onRouteChanged, expectedRouteChanged) ||
            !test_utils::testEqualValues(onRouteAdvance, expectedRouteAdvance)) {
            throw std::runtime_error("Failed");
        }
    };

    auto router = Router(
        stop,
        route_graph::Router::Options{
            .smoothRadius = 0.0,
            .maxProjectionDistance = 5.0,
            .maxAdvance = 2.0,
        },
        [&](const SharedRoutePath& path) mutable {
            curPath = path;
            onRouteChanged++;
        },
        [&](double distance, const LevelPoint& point) mutable {
            curDistance = distance;
            curPoint = point;
            onRouteAdvance++;
        });

    try {
        router.updateGraph(routeGraph);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=-1}});
        checkTest(
            "TestGetPath2.1",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            0,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}},
            1,
            1);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=1}});
        checkTest(
            "TestGetPath2.2",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            1,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=1}},
            1,
            2);


        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=3}});
        checkTest(
            "TestGetPath2.3",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            3,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=3}},
            1,
            3);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=5}});
        checkTest(
            "TestGetPath2.4",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            5,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=5}},
            1,
            4);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=7}});
        checkTest(
            "TestGetPath2.5",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            7,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=7}},
            1,
            5);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=9}});
        checkTest(
            "TestGetPath2.6",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            9,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=9}},
            1,
            6);


        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=10}});
        checkTest(
            "TestGetPath2.7",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            10,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=10}},
            1,
            7);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=10}});
        checkTest(
            "TestGetPath2.8",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            11,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=10}},
            1,
            8);
        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=9}});
        checkTest(
            "TestGetPath2.9",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            12,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=9}},
            1,
            9);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=7}});
        checkTest(
            "TestGetPath2.10",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            14,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=7}},
            1,
            10);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=5}});
        checkTest(
            "TestGetPath2.11",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            16,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=5}},
            1,
            11);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=3}});
        checkTest(
            "TestGetPath2.12",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            18,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=3}},
            1,
            12);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=1}});
        checkTest(
            "TestGetPath2.13",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            20,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=1}},
            1,
            13);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=-1}});
        checkTest(
            "TestGetPath2.14",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            22,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=(1 + std::sqrt(0.5)), .y=-std::sqrt(0.5)}},
            1,
            14);
    } catch (const std::runtime_error&) {
        return -1;
    }
    return 0;
}

int test_router_basic_test_path3()
{
    auto routeGraph = std::make_shared<RouteGraph>();
    routeGraph->addLevel(LEVEL1);
    routeGraph->addLevel(LEVEL2);

    routeGraph->addVertex(RouteGraph::Vertex{.id=1, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=2, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=10}}, .isElevation = true, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=3, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=10}}, .isElevation = true, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=4, .levelPoint=LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});

    routeGraph->addVertex(RouteGraph::Vertex{.id=5, .levelPoint=LevelPoint{.level=LEVEL2, .point=geometry::XYPoint{.x=0, .y=10}}, .isElevation = true, .name = ""});
    routeGraph->addVertex(RouteGraph::Vertex{.id=6, .levelPoint=LevelPoint{.level=LEVEL2, .point=geometry::XYPoint{.x=1, .y=10}}, .isElevation = true, .name = ""});


    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=1, .src=1, .dst=2, .weight=1.0});
    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=2, .src=2, .dst=3, .weight=1.0});
    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL1, .id=3, .src=3, .dst=4, .weight=1.0});

    routeGraph->addEdge(RouteGraph::Edge{.level=LEVEL2, .id=4, .src=5, .dst=6, .weight=0.5});

    routeGraph->addEdge(RouteGraph::Edge{.level=RouteGraph::Edge::ELEVATION_LEVEL_ID, .id=5, .src=2, .dst=5, .weight=0});
    routeGraph->addEdge(RouteGraph::Edge{.level=RouteGraph::Edge::ELEVATION_LEVEL_ID, .id=6, .src=6, .dst=3, .weight=0});

    std::cout << *routeGraph << std::endl;

    auto stop = LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=-1}};

    SharedRoutePath curPath;
    double curDistance = 0;
    LevelPoint curPoint;
    size_t onRouteChanged = 0;
    size_t onRouteAdvance = 0;

    auto checkTest = [&](
        const std::string& name,
        const SharedRoutePath& expectedPath,
        double expectedDistance,
        const LevelPoint& expectedPoint,
        size_t expectedRouteChanged,
        size_t expectedRouteAdvance)
    {
        if (!isEqual(curPath, expectedPath)) {
            std::cout << "ERROR in " << name << std::endl;
            if (curPath) {
                std::cout << "curPath: " << std::endl << *curPath << std::endl;
            }
            if (expectedPath) {
                std::cout << "expectedPath: " << std::endl << *expectedPath << std::endl;
            }
            throw std::runtime_error("Failed");
        }

        if (curPoint != expectedPoint) {
            std::cout << "ERROR in " << name << std::endl;
            std::cout << "point=" << curPoint << " expected=" << expectedPoint << std::endl;
            throw std::runtime_error("Failed");
        }

        if (curDistance != expectedDistance) {
            std::cout << "ERROR in " << name << std::endl;
            std::cout << "distance=" << curDistance << " expected=" << expectedDistance << std::endl;
            throw std::runtime_error("Failed");
        }

        if (!test_utils::testEqualValues(onRouteChanged, expectedRouteChanged) ||
            !test_utils::testEqualValues(onRouteAdvance, expectedRouteAdvance)) {
            throw std::runtime_error("Failed");
        }
    };

    auto router = Router(
        stop,
        route_graph::Router::Options{
            .smoothRadius = 0.0,
            .maxProjectionDistance = 5.0,
            .maxAdvance = 2.0,
        },
        [&](const SharedRoutePath& path) mutable {
            curPath = path;
            onRouteChanged++;
        },
        [&](double distance, const LevelPoint& point) mutable {
            curDistance = distance;
            curPoint = point;
            onRouteAdvance++;
        });

    try {
        router.updateGraph(routeGraph);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=-1}});
        checkTest(
            "TestGetPath3.1",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            0,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=0}},
            1,
            1);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=1}});
        checkTest(
            "TestGetPath3.2",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            1,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=1}},
            1,
            2);


        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=3}});
        checkTest(
            "TestGetPath3.3",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            3,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=3}},
            1,
            3);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=5}});
        checkTest(
            "TestGetPath3.4",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            5,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=5}},
            1,
            4);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=7}});
        checkTest(
            "TestGetPath3.5",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            7,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=7}},
            1,
            5);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=-1, .y=9}});
        checkTest(
            "TestGetPath3.6",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            9,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=0, .y=9}},
            1,
            6);

        router.updatePosition(LevelPoint{.level=LEVEL2, .point=geometry::XYPoint{.x=0.5, .y=11}});
        checkTest(
            "TestGetPath3.7",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            10.5,
            LevelPoint{.level=LEVEL2, .point=geometry::XYPoint{.x=0.5, .y=10}},
            1,
            7);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=10}});
        checkTest(
            "TestGetPath3.8",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            11,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=10}},
            1,
            8);
        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=9}});
        checkTest(
            "TestGetPath3.9",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            12,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=9}},
            1,
            9);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=7}});
        checkTest(
            "TestGetPath3.10",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            14,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=7}},
            1,
            10);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=5}});
        checkTest(
            "TestGetPath3.11",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            16,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=5}},
            1,
            11);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=3}});
        checkTest(
            "TestGetPath3.12",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            18,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=3}},
            1,
            12);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=1}});
        checkTest(
            "TestGetPath3.13",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            20,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=1, .y=1}},
            1,
            13);

        router.updatePosition(LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=2, .y=-1}});
        checkTest(
            "TestGetPath3.14",
            std::make_shared<RoutePath>(
                createRoutePoints({{LEVEL1, 0, 0}, {LEVEL1, 0, 10}, {LEVEL2, 0, 10}, {LEVEL2, 1, 10}, {LEVEL1, 1, 10}, {LEVEL1, 1, 0}, {LEVEL1, 2, -1}}),
                createRouteEvents({{RouteEvent::Type::TRANSITION, 2, 10}, {RouteEvent::Type::TRANSITION, 1, 11}, {RouteEvent::Type::TURN_LEFT, 45, 21}})),
            22,
            LevelPoint{.level=LEVEL1, .point=geometry::XYPoint{.x=(1 + std::sqrt(0.5)), .y=-std::sqrt(0.5)}},
            1,
            14);
    } catch (const std::runtime_error&) {
        return -1;
    }
    return 0;
}
