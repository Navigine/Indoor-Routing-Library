#include <string>
#include <vector>

#include <navigine/route_graph/route_graph.h>
#include "test_utils.h"

using namespace navigine;
using namespace navigine::route_graph;

namespace {

// taken from https://ru.wikipedia.org/wiki/Компонента_сильной_связности

//   (5) (3) (6) (6) (7)  (10) (8)
//  [0,1] → [1,1] → [2,1]  ⇆  [3,1]
//                       (11)
//
// (1)↑  (5)↙ ↓(4)    ↓(9)  (12)⇅(13)
//
//               (7)
//  [0,0] → [1,0] ⇆ [2,0]  ←  [3,1]
//   (1) (2) (2) (8) (3)  (14) (4)
//
RouteGraph buildGraph()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=2,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=4,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=3, .y=0}}, .isElevation = false, .name = ""});

    routeGraph.addVertex(RouteGraph::Vertex{.id=5,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=6,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=7,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=8,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=3, .y=1}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=1, .src=1, .dst=5, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=2, .src=1, .dst=2, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=3, .src=5, .dst=6, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=4, .src=6, .dst=2, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=5, .src=6, .dst=1, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=6, .src=6, .dst=7, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=7, .src=2, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=8, .src=3, .dst=2, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=9, .src=7, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=10, .src=7, .dst=8, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=11, .src=8, .dst=7, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=12, .src=8, .dst=4, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=13, .src=4, .dst=8, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=14, .src=4, .dst=3, .weight=1.0});

    return routeGraph;
}

RouteGraph buildSCC0()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=1,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=5,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=0, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=6,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=1}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=1, .src=1, .dst=5, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=3, .src=5, .dst=6, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=5, .src=6, .dst=1, .weight=1.0});

    return routeGraph;
}

RouteGraph buildSCC1()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=4,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=3, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=7,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=1}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=8,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=3, .y=1}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=10, .src=7, .dst=8, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=11, .src=8, .dst=7, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=12, .src=8, .dst=4, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=13, .src=4, .dst=8, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=14, .src=4, .dst=3, .weight=1.0});

    return routeGraph;
}

RouteGraph buildSCC2()
{
    RouteGraph routeGraph;
    routeGraph.addLevel(1);
    routeGraph.addVertex(RouteGraph::Vertex{.id=2,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=1, .y=0}}, .isElevation = false, .name = ""});
    routeGraph.addVertex(RouteGraph::Vertex{.id=3,.levelPoint=LevelPoint{.level=1, .point=geometry::XYPoint{.x=2, .y=0}}, .isElevation = false, .name = ""});

    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=7, .src=2, .dst=3, .weight=1.0});
    routeGraph.addEdge(RouteGraph::Edge{.level=1, .id=8, .src=3, .dst=2, .weight=1.0});

    return routeGraph;
}

} // namespace


int test_subgraph()
{
    const auto routeGraph = buildGraph();

    if (!test_utils::testEqualValues(routeGraph.buildSubGraph({1, 5, 6}), buildSCC0()) ||
        !test_utils::testEqualValues(routeGraph.buildSubGraph({4, 7, 8}), buildSCC1()) ||
        !test_utils::testEqualValues(routeGraph.buildSubGraph({2, 3}), buildSCC2())){
        return -1;
    }
    return 0;
}

int test_scc()
{
    const auto routeGraph = buildGraph();

    auto scc = routeGraph.buildSCC();

    if (!test_utils::testEqualValues(scc.size(), (size_t)3u) ||
        !test_utils::testEqualValues(scc[0], buildSCC0()) ||
        !test_utils::testEqualValues(scc[1], buildSCC1()) ||
        !test_utils::testEqualValues(scc[2], buildSCC2())) {
        return -1;
    }

    return 0;
}
