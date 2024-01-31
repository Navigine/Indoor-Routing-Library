#include <navigine/route_graph/route_graph_utils.h>

#include "test_utils.h"

using namespace navigine;
using namespace navigine::route_graph;
using namespace navigine::route_graph::helpers;

namespace navigine::route_graph::helpers {

bool operator==(const AddVertex& op1, const AddVertex& op2)
{
    return op1.point == op2.point;
}

bool operator==(const RemoveVertex& op1, const RemoveVertex& op2)
{
    return op1.index == op2.index;
}

bool operator==(const AddEdge& op1, const AddEdge& op2)
{
    return
        op1.srcIndex == op2.srcIndex &&
        op1.dstIndex == op2.dstIndex &&
        op1.type == op2.type;
}

bool operator==(const RemoveEdge& op1, const RemoveEdge& op2)
{
    return op1.index == op2.index;
}

bool operator==(const PaintVertex& op1, const PaintVertex& op2)
{
    return
        op1.index == op2.index &&
        op1.color == op2.color;
}

bool operator==(const PaintEdge& op1, const PaintEdge& op2)
{
    return
        op1.index == op2.index &&
        op1.color == op2.color;
}

} // namespace navigine::route_graph::helpers

int test_route_graph_utils_case_1()
{
    std::vector<Vertex> vertices;
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=-1, .y=0}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=1, .y=0}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=1}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=-1}});

    std::vector<Edge> edges;
    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 1, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 2, .dstIndex = 3, .type = Edge::Type::BIDIRECTIONAL});

    auto result = makeGraphCorrectionProgram(0.1, edges, vertices);

    if (!test_utils::testEqualValues(result[0], GraphOperation{AddVertex{.point{.x=0, .y=0}}}) ||
        !test_utils::testEqualValues(result[1], GraphOperation{AddEdge{.srcIndex=0, .dstIndex=4, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[2], GraphOperation{AddEdge{.srcIndex=4, .dstIndex=1, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[3], GraphOperation{RemoveEdge{.index=0}}) ||
        !test_utils::testEqualValues(result[4], GraphOperation{AddEdge{.srcIndex=2, .dstIndex=4, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[5], GraphOperation{AddEdge{.srcIndex=4, .dstIndex=3, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[6], GraphOperation{RemoveEdge{.index=1}}) ||
        !test_utils::testEqualValues(result[7], GraphOperation{PaintVertex{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[8], GraphOperation{PaintVertex{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[9], GraphOperation{PaintVertex{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[10], GraphOperation{PaintVertex{.index=3, .color=0}}) ||
        !test_utils::testEqualValues(result[11], GraphOperation{PaintVertex{.index=4, .color=0}}) ||
        !test_utils::testEqualValues(result[12], GraphOperation{PaintEdge{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[13], GraphOperation{PaintEdge{.index=3, .color=0}}) ||
        !test_utils::testEqualValues(result[14], GraphOperation{PaintEdge{.index=4, .color=0}}) ||
        !test_utils::testEqualValues(result[15], GraphOperation{PaintEdge{.index=5, .color=0}})) {
        return -1;
    }

    return 0;
}

int test_route_graph_utils_case_2()
{
    std::vector<Vertex> vertices;
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=0}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=1, .y=0}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0.999, .y=0.999}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0.999, .y=2}});

    std::vector<Edge> edges;
    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 1, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 2, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 1, .dstIndex = 3, .type = Edge::Type::BIDIRECTIONAL});

    auto result = makeGraphCorrectionProgram(0.1, edges, vertices);

    if (!test_utils::testEqualValues(result[0], GraphOperation{AddEdge{.srcIndex=1, .dstIndex=2, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[1], GraphOperation{AddEdge{.srcIndex=2, .dstIndex=3, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[2], GraphOperation{RemoveEdge{.index=2}}) ||
        !test_utils::testEqualValues(result[3], GraphOperation{PaintVertex{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[4], GraphOperation{PaintVertex{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[5], GraphOperation{PaintVertex{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[6], GraphOperation{PaintVertex{.index=3, .color=0}}) ||
        !test_utils::testEqualValues(result[7], GraphOperation{PaintEdge{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[8], GraphOperation{PaintEdge{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[9], GraphOperation{PaintEdge{.index=3, .color=0}}) ||
        !test_utils::testEqualValues(result[10], GraphOperation{PaintEdge{.index=4, .color=0}})) {
        return -1;
    }

    return 0;
}

int test_route_graph_utils_case_3()
{
    std::vector<Vertex> vertices;

    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=0}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=0.999}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=-1, .y=1}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=1, .y=1}});

    std::vector<Edge> edges;

    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 1, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 2, .dstIndex = 3, .type = Edge::Type::BIDIRECTIONAL});

    auto result = makeGraphCorrectionProgram(0.1, edges, vertices);

    if (!test_utils::testEqualValues(result[0], GraphOperation{AddEdge{.srcIndex=2, .dstIndex=1, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[1], GraphOperation{AddEdge{.srcIndex=1, .dstIndex=3, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[2], GraphOperation{RemoveEdge{.index=1}}) ||
        !test_utils::testEqualValues(result[3], GraphOperation{PaintVertex{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[4], GraphOperation{PaintVertex{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[5], GraphOperation{PaintVertex{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[6], GraphOperation{PaintVertex{.index=3, .color=0}}) ||
        !test_utils::testEqualValues(result[7], GraphOperation{PaintEdge{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[8], GraphOperation{PaintEdge{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[9], GraphOperation{PaintEdge{.index=3, .color=0}}) ) {
        return -1;
    }

    return 0;
}

int test_route_graph_utils_case_4()
{
    std::vector<Vertex> vertices;

    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=0}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=1.001}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=-1, .y=1}});
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=1, .y=1}});

    std::vector<Edge> edges;

    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 1, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 2, .dstIndex = 3, .type = Edge::Type::BIDIRECTIONAL});

    auto result = makeGraphCorrectionProgram(0.1, edges, vertices);

    if (!test_utils::testEqualValues(result[0], GraphOperation{AddVertex{.point{.x=0, .y=1}}}) ||
        !test_utils::testEqualValues(result[1], GraphOperation{AddEdge{.srcIndex=0, .dstIndex=4, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[2], GraphOperation{AddEdge{.srcIndex=4, .dstIndex=1, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[3], GraphOperation{RemoveEdge{.index=0}}) ||
        !test_utils::testEqualValues(result[4], GraphOperation{AddEdge{.srcIndex=2, .dstIndex=4, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[5], GraphOperation{AddEdge{.srcIndex=4, .dstIndex=3, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[6], GraphOperation{RemoveEdge{.index=1}}) ||
        !test_utils::testEqualValues(result[7], GraphOperation{AddEdge{.srcIndex=0, .dstIndex=1, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[8], GraphOperation{RemoveEdge{.index=2}}) ||
        !test_utils::testEqualValues(result[9], GraphOperation{RemoveEdge{.index=3}}) ||
        !test_utils::testEqualValues(result[10], GraphOperation{AddEdge{.srcIndex=2, .dstIndex=1, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[11], GraphOperation{RemoveEdge{.index=4}}) ||
        !test_utils::testEqualValues(result[12], GraphOperation{AddEdge{.srcIndex=1, .dstIndex=3, .type=Edge::Type::BIDIRECTIONAL}}) ||
        !test_utils::testEqualValues(result[13], GraphOperation{RemoveEdge{.index=5}}) ||
        !test_utils::testEqualValues(result[14], GraphOperation{RemoveVertex{.index=4}}) ||
        !test_utils::testEqualValues(result[15], GraphOperation{PaintVertex{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[16], GraphOperation{PaintVertex{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[17], GraphOperation{PaintVertex{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[18], GraphOperation{PaintVertex{.index=3, .color=0}}) ||
        !test_utils::testEqualValues(result[19], GraphOperation{PaintEdge{.index=6, .color=0}}) ||
        !test_utils::testEqualValues(result[20], GraphOperation{PaintEdge{.index=7, .color=0}}) ||
        !test_utils::testEqualValues(result[21], GraphOperation{PaintEdge{.index=8, .color=0}}) ) {
        return -1;
    }

    return 0;
}

int test_route_graph_utils_case_5()
{
    std::vector<Vertex> vertices;

    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=0, .y=0}}); // 0
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=1, .y=1}}); // 1
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=1, .y=0}}); // 2

    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=2, .y=0}}); // 3
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=2, .y=1}}); // 4
    vertices.push_back(Vertex{.point=geometry::XYPoint{.x=3, .y=0}}); // 5

    std::vector<Edge> edges;

    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 1, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 1, .dstIndex = 2, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 0, .dstIndex = 2, .type = Edge::Type::BIDIRECTIONAL});

    edges.push_back(Edge{.srcIndex = 3, .dstIndex = 4, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 3, .dstIndex = 5, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 4, .dstIndex = 5, .type = Edge::Type::BIDIRECTIONAL});
    edges.push_back(Edge{.srcIndex = 2, .dstIndex = 3, .type = Edge::Type::UNIDIRECTIONAL});

    auto result = makeGraphCorrectionProgram(0.1, edges, vertices);

    if (!test_utils::testEqualValues(result[0], GraphOperation{PaintVertex{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[1], GraphOperation{PaintVertex{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[2], GraphOperation{PaintVertex{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[3], GraphOperation{PaintVertex{.index=3, .color=1}}) ||
        !test_utils::testEqualValues(result[4], GraphOperation{PaintVertex{.index=4, .color=1}}) ||
        !test_utils::testEqualValues(result[5], GraphOperation{PaintVertex{.index=5, .color=1}}) ||
        !test_utils::testEqualValues(result[6], GraphOperation{PaintEdge{.index=0, .color=0}}) ||
        !test_utils::testEqualValues(result[7], GraphOperation{PaintEdge{.index=1, .color=0}}) ||
        !test_utils::testEqualValues(result[8], GraphOperation{PaintEdge{.index=2, .color=0}}) ||
        !test_utils::testEqualValues(result[9], GraphOperation{PaintEdge{.index=3, .color=1}}) ||
        !test_utils::testEqualValues(result[10], GraphOperation{PaintEdge{.index=4, .color=1}}) ||
        !test_utils::testEqualValues(result[11], GraphOperation{PaintEdge{.index=5, .color=1}}) ||
        !test_utils::testEqualValues(result[12], GraphOperation{PaintEdge{.index=6, .color=-1}}) ) {
        return -1;
    }

    return 0;
}
