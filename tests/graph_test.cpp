#include <navigine/route_graph/graph.h>

#include "test_utils.h"

using namespace navigine::route_graph;

namespace {

Graph buildGraph()
{
    Graph G;
    G.addVertex(1);
    G.addVertex(2);
    G.addVertex(3);
    G.addVertex(4);
    G.addVertex(5);
    G.addVertex(6);

    G.addEdge(1, 2, 7);
    G.addEdge(2, 1, 7);
    G.addEdge(1, 3, 9);
    G.addEdge(3, 1, 9);
    G.addEdge(1, 6, 14);
    G.addEdge(6, 1, 14);

    G.addEdge(2, 3, 10);
    G.addEdge(3, 2, 10);
    G.addEdge(2, 4, 15);
    G.addEdge(4, 2, 15);

    G.addEdge(3, 4, 11);
    G.addEdge(4, 3, 11);
    G.addEdge(3, 6, 2);
    G.addEdge(6, 3, 2);

    G.addEdge(4, 5, 6);
    G.addEdge(5, 4, 6);

    G.addEdge(5, 6, 9);
    G.addEdge(6, 5, 9);

    return G;
}

int test(TId start, TId end, const std::vector<TId>& pathRef, double distRef)
{
    const auto G = buildGraph();

    std::vector<TId> path;
    std::map<TId, Graph::VertexState> stMap;
    const auto dist = G.getPath(start, end, &path, &stMap);

    if (!test_utils::testEqualValues(dist, distRef) || path != pathRef) {
      return -1;
    }

    return 0;
}

} // namespace

int test_route_graph_1_5()
{
    std::vector<TId> pathRef = {1, 3, 6, 5};
    return test(1, 5, pathRef, 20);
}

int test_route_graph_1_4()
{
    std::vector<TId> pathRef = {1, 3, 4};
    return test(1, 4, pathRef, 20);
}
