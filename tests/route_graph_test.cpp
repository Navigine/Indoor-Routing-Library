#include <string>
#include <vector>

#include <navigine/route_graph/route_graph.h>
#include <navigine/geometry/utils.h>

#include "test_utils.h"

using namespace navigine;
using namespace navigine::route_graph;

namespace {

RouteGraph buildGraph()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=2}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=4, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=5,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=3, .y=3}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=6,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=3}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=1, .src=1, .dst=2, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=2, .src=2, .dst=1, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=3, .src=1, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=4, .src=3, .dst=1, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=5, .src=1, .dst=6, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=6, .src=6, .dst=1, .weight=1.0});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=7, .src=2, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=8, .src=3, .dst=2, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=9, .src=2, .dst=4, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=10, .src=4, .dst=2, .weight=1.0});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=11, .src=3, .dst=4, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=12, .src=4, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=13, .src=3, .dst=6, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=14, .src=6, .dst=3, .weight=1.0});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=15, .src=4, .dst=5, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=16, .src=5, .dst=4, .weight=1.0});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=17, .src=5, .dst=6, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=18, .src=6, .dst=5, .weight=1.0});

    return routeGraph;
}

} // namespace


int test_route_graph_get_path()
{
    const auto routeGraph = buildGraph();

    LevelPoint P{.level=1, .point=geometry::XYPoint{.x=0.5, .y=0.5}};
    LevelPoint Q{.level=1, .point=geometry::XYPoint{.x=3.5, .y=3.5}};

    const auto path = routeGraph.getPath(P, Q, {});

    if (!test_utils::testPointsCount(path->points().size(), 5u) || !test_utils::testCloseValues(path->length(), 5.4142135623)) {
        return -1;
    }

    const auto& events = path->events();

    if (!test_utils::testPointsCount(events.size(), 3u)) {
        return -1;
    }

    if (!test_utils::testPointsCount(events[0].value, 45) || !test_utils::testCloseValues(events[0].distance, 0.70710678118)) {
        return -1;
    }

    if (!test_utils::testPointsCount(events[1].value, 90) || !test_utils::testCloseValues(events[1].distance, 2.7071067811)) {
        return -1;
    }

    if (!test_utils::testPointsCount(events[2].value, 45) || !test_utils::testCloseValues(events[2].distance, 4.7071067811)) {
        return -1;
    }

    if (!test_utils::testCheckFull(routeGraph)) {
        return -1;
    }

    return 0;
}

int test_route_graph_empty_graph()
{
    RouteGraph routeGraph;

    LevelPoint P{.level=1, .point=geometry::XYPoint{.x=0.5, .y=0.5}};
    LevelPoint Q{.level=1, .point=geometry::XYPoint{.x=3.5, .y=3.5}};

    const auto path = routeGraph.getPath(P, Q, {});
    if (path != nullptr) {
        std::cout << "Empty graph should be nullptr \n";
        return -1;
    }
    return 0;
}

int test_route_graph_different_level_graph()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addLevel(2);

    routeGraph.addVertex(RouteGraph::Vertex{.id=1,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=1}}, .isElevation = true, .name = ""});

    routeGraph.addVertex(RouteGraph::Vertex{.id=3,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=3, .y=1}}, .isElevation = true, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=4, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=5,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=4, .y=2}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=6,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=3, .y=2}}, .isElevation = true, .name = ""});

    routeGraph.addVertex(RouteGraph::Vertex{.id=7,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=2}}, .isElevation = true, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=8,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=2}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=1, .src=1, .dst=2, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=RouteGraph::Edge::ELEVATION_LEVEL_ID, .id=2, .src=2, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=2, .id=3, .src=3, .dst=4, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=2, .id=4, .src=4, .dst=5, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=2, .id=5, .src=5, .dst=6, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=RouteGraph::Edge::ELEVATION_LEVEL_ID, .id=6, .src=6, .dst=7, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=7, .src=7, .dst=8, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=8, .src=8, .dst=1, .weight=1.0});


    LevelPoint P{.level=1, .point=geometry::XYPoint{.x=0.5, .y=0.5}};
    LevelPoint Q{.level=1, .point=geometry::XYPoint{.x=1, .y=3}};

    const auto path = routeGraph.getPath(P, Q, {});

    if (!test_utils::testPointsCount(path->points().size(), 10u)) {
        return -1;
    }

    if (!test_utils::testPointsCount(path->points().size(), 10u) || !test_utils::testCloseValues(path->length(), 6.7071067811)) {
        return -1;
    }

    return 0;
}

int test_route_graph_get_paths()
{
    auto routeGraph = buildGraph();

    // modified first level
    routeGraph.addVertex(RouteGraph::Vertex{.id=7,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=4, .y=4}}, .isElevation = true, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=19, .src=4, .dst=7, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=20, .src=7, .dst=4, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=21, .src=5, .dst=7, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=22, .src=7, .dst=5, .weight=1.0});

    // add second level without edges
    routeGraph.addLevel(2);
    routeGraph.addVertex(RouteGraph::Vertex{.id=8,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=3, .y=1}}, .isElevation = true, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=9,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=3, .y=2}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=10,.levelPoint=LevelPoint{.level=2, .point=geometry::XYPoint{.x=3, .y=3}}, .isElevation = false, .name = ""});

    LevelPoint P{.level=1, .point=geometry::XYPoint{.x=0.5, .y=0.5}};
    std::vector<LevelPoint> targets = {
        LevelPoint{.level=1, .point=geometry::XYPoint{.x=4, .y=4}},
        LevelPoint{.level=2, .point=geometry::XYPoint{.x=3, .y=3}}
    };

    const auto paths = routeGraph.getPaths(P, targets, {});

    if (!test_utils::testCloseValues(paths[0]->length(), 6.1213203435) || paths[1] != nullptr) {
        return -1;
    }
    return 0;
}
