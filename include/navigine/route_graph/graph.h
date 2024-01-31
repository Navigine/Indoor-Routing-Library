#pragma once

#include <cstdint>
#include <vector>
#include <optional>
#include <ostream>
#include <set>
#include <map>

#include <navigine/route_graph/utils.h>

namespace navigine::route_graph {

class Graph
{
public:
    struct VertexState
    {
        std::optional<TId> parent;
        double dist;
    };

public:
    /// Check if graph is empty
    bool isEmpty() const;

    /// Clear graph
    void clear();

    /// Add vertex to the graph.
    void addVertex(TId v);
    void removeVertex(TId v);

    /// Add edge to the graph.
    void addEdge(TId v1, TId v2, double weight = 1.0);
    void removeEdge(TId v1, TId v2);

    /// Determine the lightest path from v1 to v2.
    /// On success, function returns the overall weight of the found path,
    /// which is stored in the corresponding output variable.
    /// Otherwise, function returns the negative value.
    double getPath(TId v1, TId v2, std::vector<TId>* path = 0,
                   std::map<TId, VertexState>* stateMap = 0) const;

    std::vector<std::set<TId>> getSCC() const;

    friend std::ostream& operator<<(std::ostream& stream, const Graph& graph);

private:
    struct VertexTimes
    {
        std::optional<int> entryTime;
        std::optional<int> exitTime;
    };

    struct Edge
    {
        TId begin;
        TId end;
        double weight;

        bool operator== ( const Edge& e ) const { return begin == e.begin && end == e.end; }
        bool operator<  ( const Edge& e ) const { return begin < e.begin || (begin == e.begin && end < e.end); }
    };

    static void dfsVisit(const std::set<Edge>& edges, TId u, std::map<TId, VertexTimes>& m, int& time);

private:
    std::set<TId> vertices_;
    std::set<Edge> edges_;
};

std::ostream& operator<<(std::ostream& stream, const Graph& graph);

} // namespace navigine::route_graph
