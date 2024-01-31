
#include <string>
#include <vector>

#include <navigine/route_graph/route_graph.h>

#include "test_utils.h"

using namespace navigine;
using namespace navigine::route_graph;

int test_simplify_cross()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=-1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=-1}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=100, .src=1000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=200, .src=2000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=300, .src=3000, .dst=4000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=400, .src=4000, .dst=3000, .weight=1.0});
    routeGraph.simplify(0.1);

    unsigned edgesCount = 0;
    for (auto it = routeGraph.edgeBegin(); it != routeGraph.edgeEnd(); ++it) {
        ++edgesCount;
        auto vBegin = routeGraph.getVertex(it->src);
        auto vEnd = routeGraph.getVertex(it->dst);
        if (!test_utils::testEqualValues(geometry::GetDist(vBegin->levelPoint.point, vEnd->levelPoint.point), 1.0)) {
            return -1;
        }
    }

    if (!test_utils::testEqualValues(edgesCount, 8u)) {
        return -1;
    }

    unsigned vertexCount = 0;
    for (auto it = routeGraph.vertexBegin(); it != routeGraph.vertexEnd(); ++it) {
        ++vertexCount;
    }

    if (!test_utils::testEqualValues(vertexCount, 5u)) {
        return -1;
    }

    auto scc = routeGraph.buildSCC();

    if (!test_utils::testEqualValues(scc.size(), (size_t)1u)) {
        return -1;
    }
    return 0;
}

int test_triangle_1()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0.999, .y=0.999}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0.999, .y=2}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=100, .src=1000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=200, .src=2000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=300, .src=1000, .dst=3000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=400, .src=3000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=500, .src=4000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=600, .src=2000, .dst=4000, .weight=1.0});
    routeGraph.simplify(0.1);

    unsigned edgesCount = 0;
    for (auto it = routeGraph.edgeBegin(); it != routeGraph.edgeEnd(); ++it) {
        ++edgesCount;
    }

    if (!test_utils::testEqualValues(edgesCount, 8u)) {
        return -1;
    }

    unsigned vertexCount = 0;
    for (auto it = routeGraph.vertexBegin(); it != routeGraph.vertexEnd(); ++it) {
        ++vertexCount;
    }

    if (!test_utils::testEqualValues(vertexCount, 4u)) {
        return -1;
    }

    auto scc = routeGraph.buildSCC();

    if (!test_utils::testEqualValues(scc.size(), (size_t)1u)) {
        return -1;
    }

    return 0;
}

int test_triangle_2()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1.001, .y=0.999}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0.999, .y=2}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=100, .src=1000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=200, .src=2000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=300, .src=1000, .dst=3000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=400, .src=3000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=500, .src=4000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=600, .src=2000, .dst=4000, .weight=1.0});
    routeGraph.simplify(0.1);

    unsigned edgesCount = 0;
    for (auto it = routeGraph.edgeBegin(); it != routeGraph.edgeEnd(); ++it) {
        ++edgesCount;
    }

    if (!test_utils::testEqualValues(edgesCount, 8u)) {
        return -1;
    }

    unsigned vertexCount = 0;
    for (auto it = routeGraph.vertexBegin(); it != routeGraph.vertexEnd(); ++it) {
        ++vertexCount;
    }

    if (!test_utils::testEqualValues(vertexCount, 4u)) {
        return -1;
    }

    auto scc = routeGraph.buildSCC();

    if (!test_utils::testEqualValues(scc.size(), (size_t)1u)) {
        return -1;
    }

    return 0;
}

int test_triangle_3()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1.001, .y=2}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4000,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0.999, .y=2}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=100, .src=1000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=200, .src=2000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=300, .src=1000, .dst=3000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=400, .src=3000, .dst=1000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=500, .src=4000, .dst=2000, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=600, .src=2000, .dst=4000, .weight=1.0});
    routeGraph.simplify(0.1);

    unsigned edgesCount = 0;
    for (auto it = routeGraph.edgeBegin(); it != routeGraph.edgeEnd(); ++it) {
        ++edgesCount;
    }

    if (!test_utils::testEqualValues(edgesCount, 6u)) {
        return -1;
    }

    unsigned vertexCount = 0;
    for (auto it = routeGraph.vertexBegin(); it != routeGraph.vertexEnd(); ++it) {
        ++vertexCount;
    }

    if (!test_utils::testEqualValues(vertexCount, 3u)) {
        return -1;
    }

    auto scc = routeGraph.buildSCC();

    if (!test_utils::testEqualValues(scc.size(), (size_t)1u)) {
        return -1;
    }

    return 0;
}
