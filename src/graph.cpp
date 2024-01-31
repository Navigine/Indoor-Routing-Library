#include <algorithm>
#include <navigine/route_graph/graph.h>
#include <navigine/geometry/utils.h>

namespace navigine::route_graph {

bool Graph::isEmpty() const
{
    return vertices_.empty();
}

void Graph::clear()
{
    vertices_.clear();
    edges_.clear();
}

void Graph::addVertex(TId v)
{
    vertices_.insert(v);
}

void Graph::removeVertex(TId v)
{
    vertices_.erase(v);
    for(auto iter = edges_.begin(); iter != edges_.end(); ) {
        if (iter->begin == v || iter->end == v) {
            edges_.erase(iter++);
        } else {
            ++iter;
        }
    }
}

void Graph::addEdge(TId v1, TId v2, double weight)
{
    // Verifying edge parameters
    if (v1 == v2 ||
        vertices_.find(v1) == vertices_.end() ||
        vertices_.find(v2) == vertices_.end()) {
        return;
    }

    weight = std::max(weight, EPSILON);

    // Searching for duplicates. If duplicate edge exist,
    // replace its weight with the specified value.
    auto iter = edges_.find(Edge{.begin=v1, .end=v2, .weight=0});
    if (iter != edges_.end()) {
        Edge& e = const_cast<Edge&>(*iter);
        e.weight = weight;
        return;
    }

    edges_.insert(iter, Edge{.begin=v1, .end=v2, .weight=weight});
}

void Graph::removeEdge(TId v1, TId v2)
{
    // Verifying edge parameters
    if (v1 == v2 ||
        vertices_.find(v1) == vertices_.end() ||
        vertices_.find(v2) == vertices_.end()) {
        return;
    }

    // Searching for (v1 -> v2) edge. If such edge exist, remove it.
    auto iter = edges_.find(Edge{.begin=v1, .end=v2, .weight=0.0});
    if (iter != edges_.end()) {
        edges_.erase(iter);
    }
}

double Graph::getPath(TId v1, TId v2, std::vector<TId>* path, std::map<TId, VertexState>* stmap) const
{
    if (vertices_.find(v1) == vertices_.end() ||
        vertices_.find(v2) == vertices_.end()) {
        return -1.0;
    }

    std::set<TId> current; // stores the working set: unvisited reachable vertices
    std::map<TId, VertexState> stateMap;
    stateMap[v1] = VertexState{.parent=std::nullopt, .dist=0.0};
    current.insert(v1);

    while (true) {
        // Searching for the closest vertex...
        auto minIter = stateMap.end();
        for(auto it = current.begin(); it != current.end(); ++it) {
            auto iter = stateMap.find(*it);
            if (minIter == stateMap.end() || iter->second.dist < minIter->second.dist) {
                minIter = iter;
            }
        }

        if (minIter == stateMap.end()) {
            break; // All reachable vertice are visited => finished
        }

        const auto u = minIter->first;

        current.erase(u);

        // loop all edges outgoing from u
        for(auto eIter = edges_.lower_bound(Edge{.begin=u, .end=0, .weight=0.0});
            eIter != edges_.end() && eIter->begin == u; ++eIter)
        {
            const auto& e = *eIter;
            const double dist0 = minIter->second.dist + e.weight;

            auto iter = stateMap.find(e.end);
            if (iter != stateMap.end()) {
                if (iter->second.dist > dist0) {
                    iter->second.parent = u;
                    iter->second.dist = dist0;
                }
            } else {
                stateMap[e.end] = VertexState{.parent=u, .dist=dist0};
                current.insert(e.end);
            }
        }
    }

    if (stmap) {
        *stmap = stateMap;
    }

    if (path) {
        path->clear();
        for(std::optional<TId> v = v2; v; ) {
            auto iter = stateMap.find(*v);
            if (iter == stateMap.end()) {
                return -1.0;
            }

            path->push_back(*v);
            v = iter->second.parent;
        }
        std::reverse(path->begin(), path->end());
    }

    return stateMap[v2].dist;
}

void Graph::dfsVisit(const std::set<Edge>& edges, TId u, std::map<TId, VertexTimes>& stateMap, int& time)
{
    stateMap[u].entryTime = ++time;
    for(auto eIter = edges.lower_bound(Edge{.begin=u, .end=0, .weight=0.0});
        eIter != edges.end() && eIter->begin == u; ++eIter)
    {
        auto v = eIter->end;
        if (!stateMap[v].entryTime) {
            dfsVisit(edges, v, stateMap, time);
        }
    }
    stateMap[u].exitTime = ++time;
}

std::vector<std::set<TId>> Graph::getSCC() const
{
    // Doing DFS
    std::map<TId, VertexTimes> stateMap;
    std::vector<TId> vOrder;

    for(auto u : vertices_) {
        stateMap[u] = VertexTimes();
        vOrder.push_back(u);
    }

    int time = 0;
    for(auto u : vertices_) {
        if (!stateMap[u].entryTime) {
            dfsVisit(edges_, u, stateMap, time);
        }
    }

    // Building reverse edges
    std::set<Edge> rEdges;
    for(auto e : edges_) {
        rEdges.insert(Edge{.begin=e.end, .end=e.begin, .weight=e.weight});
    }

    // Building ordered vertex sequence
    std::sort(vOrder.begin(), vOrder.end(),
        [&stateMap](auto v1, auto v2)
        {
            return *(stateMap[v1].exitTime) > *(stateMap[v2].exitTime);
        });

    // Reinitializing stateMap
    for(auto u : vertices_) {
        stateMap[u] = VertexTimes();
    }

    // Reinitializing time
    time = 0;

    // Doing DFS for reverse graph
    for(auto u : vOrder) {
        if (!stateMap[u].entryTime) {
            dfsVisit(rEdges, u, stateMap, time);
        }
    }

    std::vector<std::optional<TId>> tree(time, std::nullopt);
    for(auto p : stateMap) {
        tree[*(p.second.entryTime) - 1] = tree[*(p.second.exitTime) - 1] = p.first;
    }

    std::vector<std::set<TId>> sccList;
    std::set<TId> scc;

    std::optional<TId> marker;

    for(size_t i = 0; i < tree.size(); ++i) {
        if (!tree[i]) {
            continue;
        }

        if (!marker) {
            marker = tree[i];
            scc.insert(*tree[i]);
            continue;
        }

        if (*tree[i] == *marker) {
            sccList.push_back(scc);
            scc.clear();
            marker = std::nullopt;
            continue;
        }

        scc.insert(*tree[i]);
    }

    return sccList;
}

std::ostream& operator<<(std::ostream& out, const Graph& graph)
{
    for(auto iter = graph.edges_.begin(); iter != graph.edges_.end(); ++iter) {
        out << "Edge " << iter->begin << " -> " << iter->end << " [weight=" << iter->weight << "]" << std::endl;
    }

    return out;
}

} // namespace navigine::route_graph
