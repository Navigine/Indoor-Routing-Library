#include <navigine/route_graph/route_graph_utils.h>
#include <navigine/route_graph/graph.h>
#include <navigine/geometry/geolib/detail/intersection.h>

namespace navigine::route_graph::helpers {

namespace {

constexpr const int MAX_ITERATIONS = 5;

struct GraphData
{
    const double snapDistance;
    std::vector<Edge> edges;
    std::vector<Vertex> vertices;
    std::vector<GraphOperation> program;
};

void addVertex(const geometry::XYPoint& point, GraphData& data)
{
    data.program.push_back(AddVertex{
        .point = point,
    });

    data.vertices.push_back(
        Vertex{
            .point = point
        }
    );
}

void addEdge(Index srcIndex, Index dstIndex, Edge::Type type, GraphData& data)
{
    data.program.push_back(AddEdge{
        .srcIndex = srcIndex,
        .dstIndex = dstIndex,
        .type = type,
    });

    data.edges.push_back(Edge{
        .srcIndex = srcIndex,
        .dstIndex = dstIndex,
        .type = type,
    });
}

void removeEdge(Index index, GraphData& data)
{
    data.program.push_back(RemoveEdge{
        .index = index,
    });

    data.edges[index].deleted = true;
}

void removeVertex(Index index, GraphData& data)
{
    data.program.push_back(RemoveVertex{
        .index = index,
    });

    data.vertices[index].deleted = true;
}

void paintVertex(Index index, Color color, GraphData& data)
{
    data.program.push_back(PaintVertex{
        .index = index,
        .color = color,
    });

    data.vertices[index].color = color;
}

void paintEdge(Index index, Color color, GraphData& data)
{
    data.program.push_back(PaintEdge{
        .index = index,
        .color = color,
    });

    data.edges[index].color = color;
}

void snapVertices(Index vi1, Index vi2, GraphData& data)
{
    const auto edgeNum = data.edges.size();
    for (Index ei = 0; ei < edgeNum; ++ei) {
        auto& e = data.edges[ei];
        if (e.deleted) {
            continue;
        }
        if ((e.srcIndex == vi1 && e.dstIndex == vi1) ||
            (e.srcIndex == vi1 && e.dstIndex == vi2) ||
            (e.srcIndex == vi2 && e.dstIndex == vi1) ||
            (e.srcIndex == vi2 && e.dstIndex == vi2)) {
            removeEdge(ei, data);
            continue;
        }

        if (e.srcIndex == vi2) {
            addEdge(vi1, e.dstIndex, e.type, data);
            removeEdge(ei, data);
        }

        if (e.dstIndex == vi2) {
            addEdge(e.srcIndex, vi1, e.type, data);
            removeEdge(ei, data);
        }
    }
    removeVertex(vi2, data);
}

bool crossEdges(GraphData& data)
{
    auto& edges = data.edges;
    auto& vertices = data.vertices;

    bool modified = false;

    for (Index ei1 = 0; ei1 < edges.size(); ++ei1) {
        if (edges[ei1].deleted) {
            continue;
        }
        const auto A = vertices[edges[ei1].srcIndex].point;
        const auto B = vertices[edges[ei1].dstIndex].point;
        for (Index ei2 = ei1 + 1; ei2 < edges.size(); ++ei2) {
            if (edges[ei2].deleted) {
                continue;
            }
            const auto C = vertices[edges[ei2].srcIndex].point;
            const auto D = vertices[edges[ei2].dstIndex].point;

            double u1;
            double v1;
            double u2;
            double v2;
            geometry::XYPoint i1;
            geometry::XYPoint i2;

            auto count = geometry::geolib::detail::segmentIntersection(A, B, C, D, u1, v1, i1, u2, v2, i2);

            if (count == 1 && 0 < u1 && u1 < 1 && 0 < v1 && v1 < 1) {
                addVertex(i1, data);
                modified = true;
            }
        }
    }

    return modified;
}

bool splitEdges(GraphData& data)
{
    auto& edges = data.edges;
    auto& vertices = data.vertices;

    bool modified = false;
    const auto edgeNum = edges.size();

    for (Index ei = 0; ei < edgeNum; ++ei) {
        if (edges[ei].deleted) {
            continue;
        }

        const auto A = vertices[edges[ei].srcIndex].point;
        const auto B = vertices[edges[ei].dstIndex].point;
        const auto orth = geometry::Vector2d(A.y - B.y, B.x - A.x).normalized();

        std::map<double, Index> splits;

        for (Index vi = 0; vi < vertices.size(); ++vi) {
            if (vertices[vi].deleted) {
                continue;
            }
            if (vi == edges[ei].srcIndex || vi == edges[ei].dstIndex) {
                continue;
            }
            const auto v = vertices[vi].point;

            const auto C = v + orth * data.snapDistance;
            const auto D = v - orth * data.snapDistance;

            double u1;
            double v1;
            double u2;
            double v2;
            geometry::XYPoint i1;
            geometry::XYPoint i2;

            auto count = geometry::geolib::detail::segmentIntersection(A, B, C, D, u1, v1, i1, u2, v2, i2);
            if (count == 1 && 0 < u1 && u1 < 1 && 0 < v1 && v1 < 1) {
                splits[u1] = vi;
                modified = true;
            }
        }

        if (splits.empty()) {
            continue;
        }

        Index srcIndex = edges[ei].srcIndex;

        for ([[maybe_unused]] const auto& [_, vi] : splits) {
            Index dstIndex = vi;
            addEdge(srcIndex, dstIndex, edges[ei].type, data);
            srcIndex = dstIndex;
        }
        addEdge(srcIndex, edges[ei].dstIndex, edges[ei].type, data);

        removeEdge(ei, data);
    }
    return modified;
}


bool snapVertices(GraphData& data)
{
    auto& vertices = data.vertices;

    bool modified = false;

    std::vector<std::pair<Index, Index>> snaps;

    for (Index vi1 = 0; vi1 < vertices.size(); ++vi1) {
        if (vertices[vi1].deleted) {
            continue;
        }
        for (Index vi2 = vi1 + 1; vi2 < vertices.size(); ++vi2) {
            if (vertices[vi2].deleted) {
                continue;
            }
            const auto v1 = vertices[vi1].point;
            const auto v2 = vertices[vi2].point;
            if (geometry::GetDist(v1, v2) < data.snapDistance) {
                snaps.emplace_back(vi1, vi2);
                modified = true;
            }
        }
    }

    for (const auto& [vi1, vi2] : snaps) {
        snapVertices(vi1, vi2, data);
    }
    return modified;
}

Graph buildGraph(GraphData& data)
{
    Graph G;

    auto& vertices = data.vertices;
    auto& edges = data.edges;

    // Adding vertice
    for (Index vi = 0; vi < vertices.size(); ++vi) {
        if (vertices[vi].deleted) {
            continue;
        }
        G.addVertex(vi);
    }

    for (Index ei = 0; ei < edges.size(); ++ei) {
        const auto& e = edges[ei];
        if (e.deleted) {
            continue;
        }

        switch (e.type) {
            case Edge::Type::UNIDIRECTIONAL:
                G.addEdge(e.srcIndex, e.dstIndex, 1.0);
                break;

            case Edge::Type::BIDIRECTIONAL:
                G.addEdge(e.srcIndex, e.dstIndex, 1.0);
                G.addEdge(e.dstIndex, e.srcIndex, 1.0);
                break;

            default:
                break;
        }
    }
    return G;
}

void paintVertices(GraphData& data)
{
    const auto graph = buildGraph(data);
    const auto sccList = graph.getSCC();

    for (size_t i = 0; i < sccList.size(); ++i) {
        for (auto vi : sccList[i]) {
            paintVertex(vi, static_cast<Color>(i), data);
        }
    }
}

void paintEdges(GraphData& data)
{
    const auto& edges = data.edges;
    const auto& vertices = data.vertices;

    for (Index ei = 0; ei < edges.size(); ++ei) {
        const auto& e = edges[ei];
        if (e.deleted) {
            continue;
        }
        const auto& src = vertices[e.srcIndex];
        const auto& dst = vertices[e.dstIndex];
        const auto color = (src.color == dst.color) ? src.color : static_cast<Color>(-1);

        paintEdge(ei, color, data);
    }
}

} // namespace

std::vector<GraphOperation> makeGraphCorrectionProgram(
    double snapDistance,
    std::vector<Edge> edges,
    std::vector<Vertex> vertices)
{
    GraphData data{
        .snapDistance = snapDistance,
        .edges = std::move(edges),
        .vertices = std::move(vertices),
        .program = {},
    };

    for(int i = 0; i < MAX_ITERATIONS; ++i) {
        bool modified = false;
        if (crossEdges(data)) {
            modified = true;
        }
        if (splitEdges(data)) {
            modified = true;
        }
        if (snapVertices(data)) {
            modified = true;
        }
        if (!modified) {
            break;
        }
    }

    paintVertices(data);
    paintEdges(data);

    return data.program;
}

std::ostream& operator<<(std::ostream& os, const GraphOperation& op)
{
    if (auto addVertex = std::get_if<AddVertex>(&op)) {
        os << "ADD_VERTEX[x=" << addVertex->point.x << ", y=" << addVertex->point.y << "]";
    } else if (auto removeVertex = std::get_if<RemoveVertex>(&op)) {
        os << "REMOVE_VERTEX[index=" << removeVertex->index << "]";
    } else if (auto addEdge = std::get_if<AddEdge>(&op)) {
        os << "ADD_EDGE[srcIndex=" << addEdge->srcIndex << ", dstIndex=" << addEdge->dstIndex << ", type=" << static_cast<int>(addEdge->type) << "]";
    } else if (auto removeEdge = std::get_if<RemoveEdge>(&op)) {
        os << "REMOVE_EDGE[index=" << removeEdge->index << "]";
    } else if (auto paintVertex = std::get_if<PaintVertex>(&op)) {
        os << "PAINT_VERTEX[index=" << paintVertex->index << ", color=" << paintVertex->color << "]";
    } else if (auto paintEdge = std::get_if<PaintEdge>(&op)) {
        os << "PAINT_EDGE[index=" << paintEdge->index << ", color=" << paintEdge->color << "]";
    }
    return os;
}

} // namespace navigine::route_graph::helpers