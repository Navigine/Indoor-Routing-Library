#pragma once

#include <navigine/route_graph/level_point.h>
#include <navigine/route_graph/route_graph.h>

namespace navigine::route_graph {

class Router {
public:
    using OnRouteChanged = std::function<void(const SharedRoutePath& path)>;
    using OnRouteAdvanced = std::function<void(double distance, const LevelPoint& point)>;

    struct Options {
        std::optional<double> smoothRadius;
        std::optional<double> maxProjectionDistance;
        std::optional<double> maxAdvance;
    };

    Router(
        const LevelPoint& finishPoint,
        const Options& options,
        OnRouteChanged onRouteChanged,
        OnRouteAdvanced onRouteAdvanced);

    void updateGraph(const std::shared_ptr<RouteGraph>& graph);

    void updatePosition(const std::optional<LevelPoint>& position);

private:
    struct RoutePosition {
        LevelPoint levelPoint;
        double advance = 0.0;
    };

    static std::optional<RoutePosition> getProjection(
        const std::vector<RoutePosition>& routePositions,
        const std::optional<RoutePosition>& currentRoutePosition,
        const std::optional<LevelPoint>& currentPosition,
        double maxProjectionDistance,
        double maxAdvance);

    void rebuildRoute();
    void update();

    void cancelRoute();

private:
    const LevelPoint finishPoint_;
    const double smoothRadius_;
    const double maxProjectionDistance_;
    const double maxAdvance_;
    const OnRouteChanged onRouteChanged_;
    const OnRouteAdvanced onRouteAdvanced_;

    std::shared_ptr<RouteGraph> graph_;
    std::optional<LevelPoint> currentPosition_;
    std::shared_ptr<RoutePath> currentPath_;
    std::vector<RoutePosition> routePositions_;
    std::optional<RoutePosition> currentRoutePosition_;
};

} // namespace navigine::route_graph