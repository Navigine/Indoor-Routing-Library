#include <algorithm>
#include <limits.h>
#include <cmath>

#include <navigine/geometry/geometry.h>
#include <navigine/geometry/utils.h>
#include <navigine/route_graph/route_graph.h>

namespace navigine::route_graph {

namespace {

constexpr const double STICK_DISTANCE = 0.1;  // meters
constexpr const int MAX_ITERATIONS = 5;

RouteGraph::Vertex CreateVertex(TId level, TId id, double x, double y, bool elevation, const std::string& name)
{
    return
        RouteGraph::Vertex{
            .id=id,
            .levelPoint=LevelPoint{.level=level, .point=geometry::XYPoint{.x=x, .y=y}},
            .isElevation=elevation,
            .name=name};
}

} // namespace

void RouteGraph::clear()
{
    *this = RouteGraph();
}

bool RouteGraph::isEmpty() const
{
    return levels_.empty() &&
           vertices_.empty() &&
           edges_.empty();
}

bool RouteGraph::addLevel(const TId& level)
{
    if (hasLevel(level)) {
        return false;
    }
    levels_.insert(level);
    return true;
}

bool RouteGraph::hasLevel(const TId& level) const
{
    return levels_.find(level) != levels_.end();
}

void RouteGraph::traverseVertices(TId levelId, TId vertexId, const VertexFunc& func) const
{
    for(auto vIter = vertices_.lower_bound(CreateVertex(levelId, vertexId, 0.0, 0.0, false, ""));
        vIter != vertices_.end() && vIter->levelPoint.level == levelId; ++vIter) {
            func(*vIter);
    }
}

void RouteGraph::traverseEdges(TId levelId, TId edgeId, const EdgeFunc& func) const
{
    for(auto eIter = edges_.lower_bound(Edge{.level=levelId, .id=edgeId, .src=0, .dst=0, .weight=0.0});
            eIter != edges_.end() && eIter->level == levelId; ++eIter) {
        func(*eIter);
    }
}

bool RouteGraph::addVertex(Vertex v)
{
    // Check if vertex already exists
    if (hasVertex(v.id)) {
        return false;
    }

    // Check if level exists
    if (!hasLevel(v.levelPoint.level)) {
        return false;
    }

    // Add vertex to RouteGraph
    auto p = vertices_.insert(v);
    vertexMap_[v.id] = p.first;
    return true;
}

void RouteGraph::removeVertex(TId id)
{
    // Check if vertex with the specified id exist
    auto iter = vertexMap_.find(id);
    if (iter == vertexMap_.end()) {
        return;
    }

    // Remove all edges incident to the specified vertex
    for(auto eIter = edgeBegin(); eIter != edgeEnd(); ) {
        if (eIter->src == id || eIter->dst == id) {
            removeEdge((eIter++)->id);
        } else {
            ++eIter;
        }
    }

    // Remove vertex from RouteGraph
    vertices_.erase(iter->second);
    vertexMap_.erase(iter);
}

void RouteGraph::moveVertex(TId id, double x, double y)
{
    // Check if vertex with the specified id exist
    auto iter = vertexMap_.find(id);
    if (iter == vertexMap_.end()) {
        return;
    }

    // Modify vertex coordinates
    // modifying const iterator, but order independent
    Vertex& P = const_cast<Vertex&>(*iter->second);
    P.levelPoint.point.x = x;
    P.levelPoint.point.y = y;
}

bool RouteGraph::hasVertex(TId id) const
{
    return vertexMap_.find(id) != vertexMap_.end();
}

void RouteGraph::setVertexName(TId id, const std::string& name)
{
    auto iter = vertexMap_.find(id);
    if (iter != vertexMap_.end()) {
        // modifying const iterator, but order independent
        Vertex& P = const_cast<Vertex&>(*(iter->second));
        P.name = name;
    }
}

void RouteGraph::setVertexElevation(TId id, bool elevation)
{
    auto iter = vertexMap_.find(id);
    if (iter != vertexMap_.end()) {
        // modifying const iterator, but order independent
        Vertex& P = const_cast<Vertex&>(*(iter->second));
        P.isElevation = elevation;
    }
}

std::optional<RouteGraph::Vertex> RouteGraph::getVertex(TId id) const
{
    auto iter = vertexMap_.find(id);
    if (iter != vertexMap_.end()) {
        return *(iter->second);
    }

    return std::nullopt;
}

RouteGraph::VertexIterator RouteGraph::vertexBegin() const
{
    return vertices_.begin();
}

RouteGraph::VertexIterator RouteGraph::vertexEnd() const
{
    return vertices_.end();
}

TId RouteGraph::getNextVertexId() const
{
    return vertexMap_.empty() ? 0 : vertexMap_.rbegin()->first + 1;
}

std::vector<RouteGraph::Vertex> RouteGraph::getElevationPoints() const
{
    std::vector<Vertex> elevationPoints;
    for(auto vIter = vertices_.begin(); vIter != vertices_.end(); ++vIter) {
        if (vIter->isElevation) {
            elevationPoints.push_back(*vIter);
        }
    }
  return elevationPoints;
}

std::vector<RouteGraph::Vertex> RouteGraph::getElevationPoints(TId level) const
{
    std::vector<Vertex> elevationPoints;
    traverseVertices(level, 0,
        [&elevationPoints](const auto& v) mutable {
            if (v.isElevation) {
                elevationPoints.push_back(v);
            }
        });

    return elevationPoints;
}

bool RouteGraph::addEdge(Edge e)
{
    // Check if edge already exists
    if (hasEdge(e.id)) {
        return false;
    }

    // Check if edge's source and destination vertice exist
    if (!hasVertex(e.src) || !hasVertex(e.dst)) {
        return false;
    }

    // Add edge to RouteGraph
    auto p = edges_.insert(e);
    edgeMap_[e.id] = p.first;
    return true;
}

void RouteGraph::removeEdge(TId id)
{
    // Check if edge with the specified id exist
    auto iter = edgeMap_.find(id);
    if (iter == edgeMap_.end()) {
        return;
    }

    // Remove edge from RouteGraph
    edges_.erase(iter->second);
    edgeMap_.erase(iter);
}

bool RouteGraph::hasEdge(TId id) const
{
    return edgeMap_.find(id) != edgeMap_.end();
}

std::optional<RouteGraph::Edge> RouteGraph::getEdge(TId id) const
{
    auto iter = edgeMap_.find(id);
    if (iter != edgeMap_.end()) {
        return *(iter->second);
    }

    return std::nullopt;
}

std::optional<RouteGraph::Edge> RouteGraph::getEdge(TId level, TId src, TId dst) const
{
    std::optional<RouteGraph::Edge> edge;

    traverseEdges(level, 0,
        [&src, &dst, &edge](const auto& e) mutable {
            if (e.src == src && e.dst == dst) {
                edge=e;
            }
        });

    return edge;
}

void RouteGraph::setEdgeWeight(TId id, double weight)
{
    auto iter = edgeMap_.find(id);
    if (iter == edgeMap_.end()) {
        return;
    }

    // modifying const iterator, but order independent
    Edge& e = const_cast<Edge&>(*(iter->second));
    e.weight = std::max(weight, EPSILON);
}

RouteGraph::EdgeIterator RouteGraph::edgeBegin() const
{
    return edges_.begin();
}

RouteGraph::EdgeIterator RouteGraph::edgeEnd() const
{
    return edges_.end();
}

TId RouteGraph::getNextEdgeId() const
{
    return edgeMap_.empty() ? 0 : edgeMap_.rbegin()->first + 1;
}

Graph RouteGraph::buildGraph() const
{
    Graph G;

    // Adding vertice
    for(const auto& v : vertices_) {
        G.addVertex(v.id);
    }

    // Adding edges
    for(const auto& e : edges_) {
        double weight = e.weight;
        if (e.level < Edge::ELEVATION_LEVEL_ID) {
            auto u = getVertex(e.src);
            auto v = getVertex(e.dst);

            assert(u);
            assert(v);

            weight *= geometry::GetDist(u->levelPoint.point, v->levelPoint.point);
        }
        G.addEdge(e.src, e.dst, weight);
    }

    // Connect close vertice
    for(const auto& v : vertices_) {
        traverseVertices(v.levelPoint.level, v.id + 1,
            [&G, &v](const auto& u) mutable {
                double dist = geometry::GetDist(v.levelPoint.point, u.levelPoint.point);

                if (dist < STICK_DISTANCE) {
                    G.addEdge(v.id, u.id, dist);
                    G.addEdge(u.id, v.id, dist);
                }
            });
    }

    // Splitting edges
    for(const auto& e : edges_) {
        if (e.level == Edge::ELEVATION_LEVEL_ID) {
            continue;
        }

        auto A = getVertex(e.src);
        auto B = getVertex(e.dst);

        assert(A);
        assert(B);

        std::vector<std::pair<double,Vertex> > splitPoints;
        splitPoints.push_back(std::make_pair(0.0, *A));
        splitPoints.push_back(std::make_pair(1.0, *B));

        traverseVertices(e.level, 0,
            [&A, &B, &splitPoints](const auto& u) mutable {
                const auto segment = geometry::XYSegment{A->levelPoint.point, B->levelPoint.point};
                const auto k = geometry::GetProjectionRatio(segment, u.levelPoint.point);
                const auto P0 = geometry::GetRatioPoint(segment, k);
                auto d = geometry::GetDist(u.levelPoint.point, P0);
                if (d < STICK_DISTANCE && 0 < k && k < 1) {
                    splitPoints.push_back(std::make_pair(k, u));
                }
            });

        // Splitting edge
        if (splitPoints.size() >= 3) {
            std::sort(splitPoints.begin(), splitPoints.end());
            for(size_t i = 1; i < splitPoints.size(); ++i) {
                double dist = geometry::GetDist(splitPoints[i-1].second.levelPoint.point, splitPoints[i].second.levelPoint.point);
                G.addEdge(splitPoints[i-1].second.id,
                          splitPoints[i].second.id,
                          dist * e.weight);
            }
        }
    }

    return G;
}

SharedRoutePath RouteGraph::getPath(LevelPoint P, LevelPoint Q, const RouteOptions& options) const
{
    RouteGraph G(*this);

    const auto P1 = getProjection(P);
    const auto Q1 = getProjection(Q);

    if (!P1 || !Q1) {
        return nullptr;
    }

    auto id = G.getNextVertexId();
    const auto pid1 = id++;
    const auto qid1 = id++;

    G.addVertex(CreateVertex(P.level, pid1, P1->point.x, P1->point.y, false, ""));
    G.addVertex(CreateVertex(Q.level, qid1, Q1->point.x, Q1->point.y, false, ""));

    const auto graph = G.buildGraph();

    std::vector<TId> ipath;
    if (graph.getPath(pid1, qid1, &ipath) > 0) {
        std::vector<LevelPoint> polyLine;
        if (options.keepFirstMile) {
            polyLine.push_back(P);
        }
        for (auto v : ipath) {
            polyLine.push_back(G.getVertex(v)->levelPoint);
        }
        if (options.keepLastMile) {
            polyLine.push_back(Q);
        }
        return RoutePath::createSmoothPath(polyLine, options.smoothRadius);
    }

    return nullptr;
}

std::vector<SharedRoutePath> RouteGraph::getPaths(
    LevelPoint P,
    const std::vector<LevelPoint>& targets,
    const RouteOptions& options) const
{
    RouteGraph G (*this);

    const auto P1 = getProjection(P);

    std::vector<SharedRoutePath> paths(targets.size(), nullptr);

    if (!P1) {
        return paths;
    }

    auto id = G.getNextVertexId();
    const auto pid1 = id++;

    G.addVertex(CreateVertex(P1->level, pid1, P1->point.x, P1->point.y, false, ""));

    std::vector<TId> qids1(targets.size());
    for(size_t i = 0; i < targets.size(); ++i) {
        qids1[i] = id++;
        const auto Q1 = getProjection(targets[i]);

        if (!Q1) {
            continue;
        }

        G.addVertex(CreateVertex(Q1->level, qids1[i], Q1->point.x, Q1->point.y, false, ""));
    }

    const auto graph = G.buildGraph();

    std::map<TId, Graph::VertexState> stateMap;
    graph.getPath(pid1, pid1, 0, &stateMap);

    for(size_t i = 0; i < targets.size(); ++i) {
        std::vector<TId> ipath;
        bool ok = true;

        for(auto v = qids1[i]; v > 0; ) {
            auto iter = stateMap.find(v);
            if (iter == stateMap.end()) {
                ok = false;
                break;
            }
            ipath.push_back(v);
            v = *(iter->second.parent);
        }

        if (!ok) {
            continue; // Route is not found
        }

        // Reversing path
        std::reverse(ipath.begin(), ipath.end());

        // Building the RoutePath
        std::vector<LevelPoint> polyLine;
        if (options.keepFirstMile) {
            polyLine.push_back(P);
        }
        for (auto v : ipath) {
            polyLine.push_back(G.getVertex(v)->levelPoint);
        }
        if (options.keepLastMile) {
            polyLine.push_back(targets[i]);
        }
        paths[i] = RoutePath::createSmoothPath(polyLine, options.smoothRadius);
    }
    return paths;
}

std::optional<double> RouteGraph::getDistance(LevelPoint P) const
{
    auto Q = getProjection(P);
    if (!Q) {
        return std::nullopt;
    }

    assert(P.level == Q->level);
    return geometry::GetDist(P.point, Q->point);
}

std::optional<LevelPoint> RouteGraph::getProjection(LevelPoint P) const
{
    std::optional<LevelPoint> P0;
    std::optional<double> d0;

    // Searching for the best edge point
    traverseEdges(P.level, 0,
        [&P, &P0, &d0, this](const auto& e) mutable {
            const auto u = getVertex(e.src);
            const auto v = getVertex(e.dst);

            assert(u);
            assert(v);

            const auto projection = geometry::GetProjection(geometry::XYSegment{u->levelPoint.point, v->levelPoint.point}, P.point);

            const auto d = projection.distance;

            if (!d0 || d < *d0) {
                P0 = LevelPoint{.level = P.level, .point = projection.point};
                d0 = d;
            }
        });

    return P0;
}

RouteGraph RouteGraph::buildSubGraph(const std::set<TId>& vertices) const
{
    auto routeGraph = RouteGraph();

    for(auto vid : vertices) {
        if (auto vertex = getVertex(vid)) {
            if (!routeGraph.hasLevel(vertex->levelPoint.level)) {
                routeGraph.addLevel(vertex->levelPoint.level);
            }
            routeGraph.addVertex(*vertex);
        }
    }

    for (auto eIter = edgeBegin(); eIter != edgeEnd(); ++eIter) {
        if (vertices.contains(eIter->src) &&
            vertices.contains(eIter->dst)) {
                routeGraph.addEdge(*eIter);
            }
    }

    return routeGraph;
}

std::vector<RouteGraph> RouteGraph::buildSCC() const
{
    std::vector<RouteGraph> result;
    const auto graph = buildGraph();

    // Check level connectivity
    for(auto scc : graph.getSCC()) {
        if (scc.empty()) {
            continue;
        }
        result.emplace_back(buildSubGraph(scc));
    }
    return result;
}

void RouteGraph::snapVertices(TId vid1, TId vid2)
{
    auto v1Iter = vertexMap_.find(vid1);
    auto v2Iter = vertexMap_.find(vid2);

    if (v1Iter == vertexMap_.end() ||
        v2Iter == vertexMap_.end() ||
        v1Iter->second->levelPoint.level != v2Iter->second->levelPoint.level) {
        return;
    }

    auto& v1 = const_cast<Vertex&>(*(v1Iter->second));
    auto& v2 = const_cast<Vertex&>(*(v2Iter->second));

    v1.isElevation = (v1.isElevation || v2.isElevation);
    v1.name = (!v1.name.empty()) ? v1.name : v2.name;

    for(auto eIter = edgeBegin(); eIter != edgeEnd(); ) {
        if ((eIter->src == vid1 && eIter->dst == vid1) ||
            (eIter->src == vid1 && eIter->dst == vid2) ||
            (eIter->src == vid2 && eIter->dst == vid1) ||
            (eIter->src == vid2 && eIter->dst == vid2)) {
            removeEdge((eIter++)->id);
            continue;
        }

        if (eIter->src == vid2) {
            auto& e = const_cast<Edge&>(*eIter);
            e.src = vid1;
        }

        if (eIter->dst == vid2) {
            auto& e = const_cast<Edge&>(*eIter);
            e.dst = vid1;
        }
        ++eIter;
    }

    // Remove vertex from RouteGraph
    vertices_.erase(v2Iter->second);
    vertexMap_.erase(v2Iter);
}

void RouteGraph::splitEdge(TId eid, const std::vector<std::tuple<double, TId, geometry::XYPoint>>& splitPoints)
{
    auto edge = getEdge(eid);
    if (!edge) {
        return;
    }

    auto beginId = edge->src;

    for(size_t i = 0; i < splitPoints.size(); ++i) {
        const auto vid = std::get<1>(splitPoints[i]);
        const auto point = std::get<2>(splitPoints[i]);

        moveVertex(vid, point.x, point.y);

        addEdge(Edge{
            .level = edge->level,
            .id = getNextEdgeId(),
            .src = beginId,
            .dst = vid,
            .weight = edge->weight
        });

        beginId = vid;
    }

    addEdge(Edge{
        .level = edge->level,
        .id = getNextEdgeId(),
        .src = beginId,
        .dst = edge->dst,
        .weight = edge->weight
    });

    removeEdge(eid);
}

void RouteGraph::crossEdges(TId eid1, TId eid2, const geometry::XYPoint& crossPoint)
{
    auto edge1 = getEdge(eid1);
    auto edge2 = getEdge(eid2);
    if (!edge1 || !edge2) {
        return;
    }

    assert(edge1->level == edge2->level);
    assert(edge1->level != Edge::ELEVATION_LEVEL_ID);

    const auto vid = getNextVertexId();

    addVertex(Vertex{
        .id = vid,
        .levelPoint = LevelPoint{
            .level = edge1->level,
            .point = crossPoint,
        },
        .isElevation = false,
        .name = "",
    });

    auto splitEdge = [vid, this](const auto& edge) {
        addEdge(Edge{
            .level = edge->level,
            .id = getNextEdgeId(),
            .src = edge->src,
            .dst = vid,
            .weight = edge->weight
        });

        addEdge(Edge{
            .level = edge->level,
            .id = getNextEdgeId(),
            .src = vid,
            .dst = edge->dst,
            .weight = edge->weight
        });
    };

    splitEdge(edge1);
    splitEdge(edge2);

    removeEdge(eid1);
    removeEdge(eid2);
}

bool RouteGraph::snapVertices(double snapDistance)
{
    std::vector<std::pair<TId, TId>> snaps;
    for(const auto& v : vertices_) {
        traverseVertices(v.levelPoint.level, v.id + 1,
            [&v, &snaps, snapDistance](const auto& u) mutable {
                if (geometry::GetDist(v.levelPoint.point, u.levelPoint.point) < snapDistance) {
                    snaps.emplace_back(v.id, u.id);
                }
            });
    }

    bool modified = false;

    // Snapping close vertice
    for (const auto& [v1, v2] : snaps) {
        snapVertices(v1, v2);
        modified = true;
    }

    return modified;
}

bool RouteGraph::splitEdges(double snapDistance)
{
    struct SplitEdgeParams
    {
        TId eid;
        std::vector<std::tuple<double, TId, geometry::XYPoint>> splitPoints;
    };
    std::vector<SplitEdgeParams> splits;

    // Snapping vertex to edges
    for(const auto& e : edges_) {
        if (e.level == Edge::ELEVATION_LEVEL_ID) {
            continue;
        }

        auto A = getVertex(e.src);
        auto B = getVertex(e.dst);

        assert(A);
        assert(B);

        const auto segment = geometry::XYSegment{A->levelPoint.point, B->levelPoint.point};

        std::vector<std::tuple<double, TId, geometry::XYPoint>> splitPoints;

        traverseVertices(e.level, 0,
            [&splitPoints, &segment, &e, snapDistance](const auto& u) mutable {
                if (u.id == e.src || u.id == e.dst) {
                    return;
                }
                const auto k = geometry::GetProjectionRatio(segment, u.levelPoint.point);
                const auto P0 = geometry::GetRatioPoint(segment, k);
                auto d = geometry::GetDist(u.levelPoint.point, P0);
                if (d < snapDistance && 0 < k && k < 1) {
                    splitPoints.push_back(std::make_tuple(k, u.id, P0));
                }
            });

        // Splitting edge
        if (!splitPoints.empty()) {
            std::stable_sort(splitPoints.begin(), splitPoints.end(),
                [](const auto& lhs, const auto& rhs){
                    return std::get<0>(lhs) < std::get<0>(rhs);
                });

            splits.push_back(SplitEdgeParams{
                .eid = e.id,
                .splitPoints = std::move(splitPoints),
            });
        }
    }

    bool modified = false;

    for (const auto& split : splits) {
        splitEdge(split.eid, split.splitPoints);
        modified = true;
    }
    return modified;
}

bool RouteGraph::crossEdges()
{
    struct CrossEdgeParams
    {
        TId eid1;
        TId eid2;
        geometry::XYPoint crossPoint;
    };
    std::vector<CrossEdgeParams> crosses;

    // Crossing edges
    for(const Edge& e1 : edges_) {
        if (e1.level == Edge::ELEVATION_LEVEL_ID) {
            continue;
        }

        auto A = getVertex(e1.src);
        auto B = getVertex(e1.dst);

        assert(A);
        assert(B);

        traverseEdges(e1.level, e1.id + 1,
            [&A, &B, &e1, &crosses, this](const Edge& e2) mutable {
                if (e1.src == e2.src || e1.src == e2.dst ||
                    e1.dst == e2.src || e1.dst == e2.dst) {
                    return;
                }
                auto C = getVertex(e2.src);
                auto D = getVertex(e2.dst);

                assert(C);
                assert(D);

                double k1;
                double k2;

                auto point = geometry::SegmentIntersection(
                    A->levelPoint.point,
                    B->levelPoint.point,
                    C->levelPoint.point,
                    D->levelPoint.point,
                    k1, k2);

                if (point && 0 < k1 && k1 < 1 && 0 < k2 && k2 < 1) {
                    crosses.push_back(CrossEdgeParams{
                        .eid1 = e1.id,
                        .eid2 = e2.id,
                        .crossPoint = *point,
                    });
                }
        });
    }

    bool modified = false;

    for (const auto& cross : crosses) {
        crossEdges(cross.eid1, cross.eid2, cross.crossPoint);
        modified = true;
    }
    return modified;
}

void RouteGraph::simplify(double snapDistance)
{
    for(int i = 0; i < MAX_ITERATIONS; ++i) {
        bool modified = false;
        if (crossEdges()) {
            modified = true;
        }
        if (splitEdges(snapDistance)) {
            modified = true;
        }
        if (snapVertices(snapDistance)) {
            modified = true;
        }
        if (!modified) {
            break;
        }
    }
}

int RouteGraph::checkCloseVertice(std::vector<std::string>* errors) const
{
    int errNum = 0;
    for(auto v : vertices_) {

        traverseVertices(v.levelPoint.level, v.id + 1,
            [&v, &errNum, &errors](const auto& u) mutable {
                auto d = geometry::GetDist(v.levelPoint.point, u.levelPoint.point);
                if (d < STICK_DISTANCE) {
                    if (++errNum && errors) {
                        std::stringstream ss;

                        ss << "<type=\"close_vertice\" level=\"" << v.levelPoint.level << "\" " <<
                                    "id1=\"" << v.id << "\" x1=\"" << v.levelPoint.point.x << "\" y1=\""<< v.levelPoint.point.y <<"\" "
                                    "id2=\"" << u.id << "\" x2=\"" << u.levelPoint.point.x << "\" y2=\"" << u.levelPoint.point.y <<"\"/>";
                        errors->push_back(ss.str());
                    }
                }
            });
    }
    return errNum;
}

int RouteGraph::checkSplitEdges(std::vector<std::string>* errors) const
{
    int errNum = 0;
    for(auto v : vertices_) {
        traverseEdges(v.levelPoint.level, 0,
            [&v, &errNum, &errors, this](const auto& e) mutable {
                auto A = getVertex(e.src);
                auto B = getVertex(e.dst);

                assert(A);
                assert(B);

                auto d1 = geometry::GetDist(v.levelPoint.point, A->levelPoint.point);
                auto d2 = geometry::GetDist(v.levelPoint.point, B->levelPoint.point);

                // Check if v is close to A or B
                if (v.id == A->id || v.id == B->id || d1 < STICK_DISTANCE || d2 < STICK_DISTANCE) {
                    return;
                }

                const auto d3 = geometry::GetProjection(geometry::XYSegment{A->levelPoint.point, B->levelPoint.point}, v.levelPoint.point).distance;

                if (d3 < STICK_DISTANCE) {
                    if (++errNum && errors) {
                        std::stringstream ss;
                        ss <<  "<type=\"split_edge\" level=\"" << v.levelPoint.level << "\" " <<
                                    "id1=\"" << v.id << "\" x1=\"" << v.levelPoint.point.x << "\" y1=\"" << v.levelPoint.point.y << "\" " <<
                                    "id2=\"" << A->id << "\" x2=\"" << A->levelPoint.point.x << "\" y2=\"" << A->levelPoint.point.y << "\" " <<
                                    "id3=\"" << B->id << "\" x3=\"" << B->levelPoint.point.x << "\" y3=\"" << B->levelPoint.point.y <<"\"/>";
                        errors->push_back(ss.str());
                    }
                }
            });
    }
    return errNum;
}

int RouteGraph::checkCrossEdges(std::vector<std::string>* errors) const
{
    int errNum = 0;
    for(auto e : edges_) {
        auto A = getVertex(e.src);
        auto B = getVertex(e.dst);

        assert(A);
        assert(B);

        if (e.level == Edge::ELEVATION_LEVEL_ID) {
            continue;
        }

        traverseEdges(e.level, e.id + 1,
            [&A, &B, &errors, &errNum, this](const auto& e) mutable {
                auto C = getVertex(e.src);
                auto D = getVertex(e.dst);

                assert(C);
                assert(D);

                if (geometry::CheckIntersection(
                    geometry::XYSegment{A->levelPoint.point, B->levelPoint.point},
                    geometry::XYSegment{C->levelPoint.point, D->levelPoint.point})) {

                    if (++errNum && errors) {
                        std::stringstream ss;
                        ss <<  "<type=\"cross_edges\" level=\"" << A->levelPoint.level << "\" " <<
                                "id1=\"" << A->id << "\" x1=\"" << A->levelPoint.point.x << "\" y1=\"" << A->levelPoint.point.y << "\" " <<
                                "id2=\"" << B->id << "\" x2=\"" << B->levelPoint.point.x << "\" y2=\"" << B->levelPoint.point.y << "\" " <<
                                "id3=\"" << C->id << "\" x3=\"" << C->levelPoint.point.x << "\" y3=\"" << C->levelPoint.point.y << "\" " <<
                                "id4=\"" << D->id << "\" x4=\"" << D->levelPoint.point.x << "\" y4=\"" << D->levelPoint.point.y << "\"/>";
                        errors->push_back(ss.str());
                    }
                }
            });
    }
    return errNum;
}

// Check connectivity inside the levels
int RouteGraph::checkLevelConnectivity(std::vector<std::string>* errors) const
{
    int errNum = 0;
    for(auto level : levels_) {
        Graph G;

        int vCount = 0;

        traverseVertices(level, 0,
            [&vCount, &G](const auto& u) mutable {
                G.addVertex(u.id);
                ++vCount;
            });

        int eCount = 0;

        traverseEdges(level, 0,
            [&G, &eCount](const auto& e) mutable {
                G.addEdge(e.src, e.dst, 1);
                ++eCount;
            });

        if (!vCount) {
            if (++errNum && errors) {
                std::stringstream ss;
                ss << "<type=\"level_no_vertice\" level=\"" << level <<"\"/>";
                errors->push_back(ss.str());
            }
            continue;
        }

        if (!eCount) {
            if (++errNum && errors) {
                std::stringstream ss;
                ss << "<type=\"level_no_edges\" level=\"" << level << "\"/>";
                errors->push_back(ss.str());
            }
            continue;
        }

        // Check level connectivity
        auto sccList = G.getSCC();
        if (sccList.size() > 1) {
            for(size_t i = 1; i < sccList.size(); ++i) {
                for(size_t j = i - 1; j < i; ++j) {
                    if (sccList[i].empty() || sccList[j].empty()) {
                        continue;
                    }

                    // Searching the closest unreachable vertex std::pair
                    Vertex v1, v2;
                    std::optional<double> d0;

                    for(auto i1 : sccList[i]) {
                        for(auto i2 : sccList[j]) {
                            auto u1 = getVertex(i1);
                            auto u2 = getVertex(i2);

                            assert(u1);
                            assert(u2);

                            double d = geometry::GetDist(u1->levelPoint.point, u2->levelPoint.point);

                            if (!d0 || d < *d0) {
                                d0 = d;
                                v1 = *u1;
                                v2 = *u2;
                            }
                        }

                        if (++errNum && errors) {
                            std::stringstream ss;
                            ss << "<type=\"level_not_connected\" level=\"" << level << "\" " <<
                                    "id1=\"" << v1.id << "\" x1=\"" << v1.levelPoint.point.x << "\" y1=\"" << v1.levelPoint.point.y << "\" " <<
                                    "id2=\"" << v2.id << "\" x2=\"" << v2.levelPoint.point.x << "\" y2=\"" << v2.levelPoint.point.y << "\"/>";
                            errors->push_back(ss.str());
                        }
                    }
                }
            }
        }
    }
    return errNum;
}

int RouteGraph::checkElevationConnectivity(std::vector<std::string>* errors) const
{
    if (levels_.size() < 2) {
        return 0;
    }

    int errNum = 0;

    Graph G;
    for(auto level : levels_) {
        G.addVertex(level);
    }

    int eCount = 0;
    traverseEdges(Edge::ELEVATION_LEVEL_ID, 0,
        [&G, &eCount, this](const auto& e) mutable {
            auto u = getVertex(e.src);
            auto v = getVertex(e.dst);

            assert(u);
            assert(v);

            G.addEdge(u->levelPoint.level, v->levelPoint.level, 1);
            ++eCount;
        });

    auto sccList = G.getSCC();
    if (sccList.size() > 1) {
        for(size_t i = 1; i < sccList.size(); ++i) {
            for(size_t j = i - 1; j < i; ++j) {
                if (sccList[i].empty() || sccList[j].empty()) {
                    continue;
                }

                for(auto id1 : sccList[i]) {
                    for(auto id2 : sccList[j]) {
                        if (++errNum && errors) {
                            std::stringstream ss;
                            ss << "<type=\"levels_not_connected\" level1=\"" << id1 << "\" level2=\"" << id2 << "\"/>";
                            errors->push_back(ss.str());
                        }
                    }
                }
            }
        }
    }

    return errNum;
}

int RouteGraph::checkFullConnectivity(std::vector<std::string>* errors) const
{
    int errNum = 0;

    if (levels_.size() > 1) {
        for(auto level : levels_) {
            Graph G;

            int vCount = 0, extCount = 0;

            traverseVertices(level, 0,
                [&vCount, &extCount](const auto& u) mutable {
                   ++vCount;
                    if (u.isElevation) {
                        ++extCount;
                    }
                });

            if (vCount && !extCount) {
                if (++errNum && errors) {
                    std::stringstream ss;
                    ss <<  "<type=\"level_no_elevations\" level=\"" << level << "\"/>";
                    errors->push_back(ss.str());
                }
            }
        }
    }

    if (errNum > 0) {
        return errNum;
    }

    // Check the graph connectivity
    Graph G;
    for(auto v : vertices_) {
        G.addVertex(v.id);
    }

    for(auto e : edges_) {
        G.addEdge(e.src, e.dst, 1);
    }

    auto sccList = G.getSCC();
    if (sccList.size() > 1) {
        for(size_t i = 1; i < sccList.size() - 1; ++i)
            for(size_t j = i - 1; j < i; ++j) {
                if (sccList[i].empty() || sccList[j].empty()) {
                    continue;
                }

                auto v1 = getVertex(*sccList[i].begin());
                auto v2 = getVertex(*sccList[j].begin());

                assert(v1);
                assert(v2);

                for(auto i1 : sccList[i]) {
                    auto u1 = getVertex(i1);
                    if (u1->isElevation) {
                        v1 = u1;
                    }
                }

                for(auto i2 : sccList[j]) {
                    auto u2 = getVertex(i2);
                    if (u2->isElevation) {
                        v2 = u2;
                    }
                }

            if (++errNum && errors) {
                std::stringstream ss;
                ss << "<type=\"graph_not_connected\" "
                        "level1=\"" << v1->levelPoint.level << "\" id1=\"" << v1->id << "\" "
                        "x1=\"" << v1->levelPoint.point.x  << "\" y1=\"" << v1->levelPoint.point.y << "\" "
                        "level2=\"" << v2->levelPoint.level << "\" id2=\"" << v2->id << "\" "
                        "x2=\"" << v2->levelPoint.point.x << "\" y2=\"" << v2->levelPoint.point.y << "\"/>";
                errors->push_back(ss.str());
            }
        }
    }

    return errNum;
}

int RouteGraph::checkFull(std::vector<std::string>* errors) const
{
    // Check if graph is empty (has no vertice)
    if (vertices_.empty()) {
        if (errors) {
            std::stringstream ss;
            ss << "<type=\"graph_empty\"/>";
            errors->push_back(ss.str());
        }
        return 1;
    }

    int n1 = checkCloseVertice(errors);
    int n2 = checkSplitEdges(errors);
    int n3 = checkCrossEdges(errors);
    int n4 = checkLevelConnectivity(errors);
    int n5 = checkElevationConnectivity(errors);
    if (n5 == 0) {
        n5 = checkFullConnectivity(errors);
    }
    return n1 + n2 + n3 + n4 + n5;
}

std::ostream& operator<<(std::ostream& out, const RouteGraph::Vertex& v)
{
    out << "Vertex[id=" << v.id << ", level=" << v.levelPoint.level << ", x=" << v.levelPoint.point.x << ", y=" << v.levelPoint.point.y << ", isElevation=" << v.isElevation << ", name=\"" << v.name << "\"]";
    return out;
}

std::ostream& operator<<(std::ostream& out, const RouteGraph::Edge& e)
{
    out << "Edge[id=" << e.id;
    if (e.level == RouteGraph::Edge::ELEVATION_LEVEL_ID) {
        out << ", ELEVATION";
    } else {
        out << ", level=" << e.level;
    }
    out << ", src=" << e.src << ", dst=" << e.dst << ", weight=" << e.weight << "]";
    return out;
}

std::ostream& operator<<(std::ostream& out, const RouteGraph& routeGraph)
{
    for(const auto& level : routeGraph.levels_) {
        out << "Level[id=" << level << "]" << std::endl;
        out << "  Vertices:" << std::endl;
        routeGraph.traverseVertices(level, 0,
            [&out](const auto& v) {
                out << "    " << v << std::endl;
            });

        out << "  Edges:" << std::endl;
        routeGraph.traverseEdges(level, 0,
            [&out](const auto& e) {
               out << "    " << e << std::endl;
            });
        out << std::endl;
    }

    out << "External edges:" << std::endl;
    routeGraph.traverseEdges(RouteGraph::Edge::ELEVATION_LEVEL_ID, 0,
        [&out](const auto& e) {
            out << "    " << e << std::endl;
        });

    return out;
}

bool operator==(const RouteGraph& routeGraph1, const RouteGraph& routeGraph2)
{
    return routeGraph1.levels_ == routeGraph2.levels_ &&
           routeGraph1.vertices_ == routeGraph2.vertices_ &&
           routeGraph1.edges_ == routeGraph2.edges_;
}

} // namespace navigine::route_graph
