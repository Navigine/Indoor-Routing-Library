#pragma once

#include <map>
#include <limits>
#include <optional>
#include <set>
#include <string>

#include <navigine/route_graph/graph.h>
#include <navigine/route_graph/route_path.h>
#include <navigine/route_graph/utils.h>

namespace navigine::route_graph {

struct RouteOptions
{
    double smoothRadius = 0;
    bool keepFirstMile = true;
    bool keepLastMile = true;
};

class RouteGraph
{
public:
    struct Vertex
    {
        TId id;
        LevelPoint levelPoint;
        bool isElevation;
        std::string name;

        bool operator==(const Vertex& v) const { return levelPoint.level && id == v.id; }
        bool operator<(const Vertex& v) const { return levelPoint.level < v.levelPoint.level || (levelPoint.level == v.levelPoint.level && id < v.id); }
    };

public:
    struct Edge
    {
        static const auto ELEVATION_LEVEL_ID = std::numeric_limits<TId>::max();
        TId level;
        TId id;
        TId src;
        TId dst;
        double weight;

        bool operator== ( const Edge& e ) const { return level == e.level && id == e.id; }
        bool operator<  ( const Edge& e ) const { return level < e.level || (level == e.level && id < e.id); }
    };

public:
    using VertexIterator = std::set<RouteGraph::Vertex>::const_iterator;
    using EdgeIterator = std::set<RouteGraph::Edge>::const_iterator;
    using VertexFunc = std::function<void(const Vertex&)>;
    using EdgeFunc = std::function<void(const Edge&)>;

public:
    RouteGraph() = default;

    // Clear/empty graph
    void clear();
    bool isEmpty() const;

    // Add/get Level
    bool addLevel(const TId& level);
    bool hasLevel(const TId& level) const;

    // Manipulating vertex
    bool addVertex(Vertex v);
    void removeVertex(TId id);
    void moveVertex(TId id, double x, double y);
    bool hasVertex(TId id) const;
    void setVertexName(TId id, const std::string& name);
    void setVertexElevation(TId id, bool elevation);
    std::optional<Vertex> getVertex(TId id) const;
    VertexIterator vertexBegin() const;
    VertexIterator vertexEnd() const;
    TId getNextVertexId() const;

    std::vector<Vertex> getElevationPoints() const;
    std::vector<Vertex> getElevationPoints(TId level) const;

    // Manipulating edge
    bool addEdge(Edge e);
    void removeEdge(TId id);
    bool hasEdge(TId id) const;
    std::optional<Edge> getEdge(TId id) const;
    std::optional<Edge> getEdge(TId level, TId src, TId dst) const;
    void setEdgeWeight(TId id, double weight);
    EdgeIterator edgeBegin() const;
    EdgeIterator edgeEnd() const;
    TId getNextEdgeId() const;

    friend std::ostream& operator<<(std::ostream& stream, const RouteGraph& routeGraph);
    friend bool operator==(const RouteGraph& routeGraph1, const RouteGraph& routeGraph2);

    RouteGraph buildSubGraph(const std::set<TId>& vertices) const;

    std::vector<RouteGraph> buildSCC() const;

    void simplify(double snapDistance);

public:
    std::optional<double> getDistance(LevelPoint P) const;

    std::optional<LevelPoint> getProjection(LevelPoint P) const;

    SharedRoutePath getPath(LevelPoint P, LevelPoint Q, const RouteOptions& options) const;

    std::vector<SharedRoutePath> getPaths(
        LevelPoint P,
        const std::vector<LevelPoint>& targets,
        const RouteOptions& options) const;

public:
    int checkCloseVertice(std::vector<std::string>* errors = nullptr) const;
    int checkSplitEdges(std::vector<std::string>* errors = nullptr) const;
    int checkCrossEdges(std::vector<std::string>* errors = nullptr) const;
    int checkLevelConnectivity(std::vector<std::string>* errors = nullptr) const;
    int checkElevationConnectivity(std::vector<std::string>* errors = nullptr) const;
    int checkFullConnectivity(std::vector<std::string>* errors = nullptr) const;
    int checkFull(std::vector<std::string>* errors = nullptr) const;

private:
    Graph buildGraph() const;

    void traverseVertices(TId levelId, TId vertexId, const VertexFunc& func) const;
    void traverseEdges(TId levelId, TId edgeId, const EdgeFunc& func) const;

    bool snapVertices(double snapDistance);
    bool splitEdges(double snapDistance);
    bool crossEdges();

    void snapVertices(TId v1, TId v2);
    void splitEdge(TId eid, const std::vector<std::tuple<double, TId, geometry::XYPoint>>& splitPoints);
    void crossEdges(TId eid1, TId eid2, const geometry::XYPoint& crossPoint);

private:
    std::set<TId> levels_;
    std::set<Vertex> vertices_;
    std::set<Edge> edges_;
    std::map<TId, VertexIterator> vertexMap_; // {vertex.id -> VertexIterator}
    std::map<TId, EdgeIterator> edgeMap_;     // {edge.id -> EdgeIterator}
};

std::ostream& operator<<(std::ostream& out, const RouteGraph::Vertex& v);
std::ostream& operator<<(std::ostream& out, const RouteGraph::Edge& e);
std::ostream& operator<<(std::ostream& stream, const RouteGraph& routeGraph);

} // namespace navigine::route_graph
