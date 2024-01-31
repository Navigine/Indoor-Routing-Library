#pragma once

#include <navigine/route_graph/utils.h>
#include <navigine/geometry/geometry.h>
#include <variant>

namespace navigine::route_graph::helpers {

using Index = size_t;
using Color = int;

struct Edge
{
    enum class Type {
        UNIDIRECTIONAL,
        BIDIRECTIONAL
    };

    Index srcIndex;
    Index dstIndex;
    Type type;
    Color color = 0;
    bool deleted = false;
};

struct Vertex
{
    geometry::XYPoint point;
    Color color = 0;
    bool deleted = false;
};

struct AddVertex
{
    geometry::XYPoint point;
};

struct RemoveVertex
{
    Index index;
};

struct AddEdge
{
    Index srcIndex;
    Index dstIndex;
    Edge::Type type;
};

struct RemoveEdge
{
    Index index;
};

struct PaintVertex
{
    Index index;
    Color color;
};

struct PaintEdge
{
    Index index;
    Color color;
};

using GraphOperation = std::variant<AddVertex, RemoveVertex, AddEdge, RemoveEdge, PaintVertex, PaintEdge>;

std::vector<GraphOperation> makeGraphCorrectionProgram(
    double snapDistance,
    std::vector<Edge> edges,
    std::vector<Vertex> vertices);

std::ostream& operator<<(std::ostream& os, const GraphOperation& op);

} // namespace navigine::route_graph::helpers