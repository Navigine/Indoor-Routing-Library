#include <navigine/route_graph/router.h>

namespace navigine::route_graph {

namespace {
constexpr double DEFAULT_SMOOTH_RADIUS = 0.0;
constexpr double DEFAULT_MAX_PROJECTION_DISTANCE = 5.0;
constexpr double DEFAULT_MAX_ADVANCE = 2.0;
}

Router::Router(
    const LevelPoint& finishPoint,
    const Options& options,
    OnRouteChanged onRouteChanged,
    OnRouteAdvanced onRouteAdvanced)
    : finishPoint_(finishPoint)
    , smoothRadius_(options.smoothRadius ? *options.smoothRadius : DEFAULT_SMOOTH_RADIUS)
    , maxProjectionDistance_(options.maxProjectionDistance ? *options.maxProjectionDistance : DEFAULT_MAX_PROJECTION_DISTANCE)
    , maxAdvance_(options.maxAdvance ? *options.maxAdvance : DEFAULT_MAX_ADVANCE)
    , onRouteChanged_(std::move(onRouteChanged))
    , onRouteAdvanced_(std::move(onRouteAdvanced))
{}

void Router::updateGraph(const std::shared_ptr<RouteGraph>& graph)
{
    graph_ = graph;
    rebuildRoute();
}

void Router::updatePosition(const std::optional<LevelPoint>& position)
{
    currentPosition_ = position;

    if (!currentRoutePosition_) {
        rebuildRoute();
        return;
    }

    auto currentRoutePosition = getProjection(
        routePositions_,
        currentRoutePosition_,
        currentPosition_,
        maxProjectionDistance_,
        maxAdvance_);

    if (!currentRoutePosition) {
        rebuildRoute();
        return;
    }

    if (currentRoutePosition_->advance != currentRoutePosition->advance) {
        currentRoutePosition_ = std::move(currentRoutePosition);
        onRouteAdvanced_(currentRoutePosition_->advance, currentRoutePosition_->levelPoint);
    }
}

/// -------------------------------- private methods -----------------------------------------

void Router::cancelRoute()
{
    currentPath_ = nullptr;
    routePositions_.clear();
    if (currentRoutePosition_) {
        currentRoutePosition_ = std::nullopt;
        onRouteChanged_(currentPath_);
    }
}

void Router::rebuildRoute()
{
    if (!currentPosition_ || !graph_) {
        cancelRoute();
        return;
    }

    auto path = graph_->getPath(
        *currentPosition_,
        finishPoint_,
        RouteOptions{
            .smoothRadius = smoothRadius_,
            .keepFirstMile = false,
            .keepLastMile = true,
        });

    if (!path) {
        cancelRoute();
        return;
    }

    std::vector<RoutePosition> routePositions;

    const auto& routePoints = path->points();

    assert(!routePoints.empty());

    routePositions.push_back(
        RoutePosition{.levelPoint = routePoints[0], .advance = 0});

    double advance = 0;
    for (size_t i = 1; i < routePoints.size(); ++i) {
        const auto& prev = routePoints[i - 1];
        const auto& curr = routePoints[i];
        if (prev.level == curr.level) {
            advance += geometry::GetDist(prev.point, curr.point);
        }
        routePositions.push_back(
            RoutePosition{.levelPoint = curr, .advance = advance});
    }

    auto currentRoutePosition = getProjection(
        routePositions,
        std::nullopt,
        currentPosition_,
        maxProjectionDistance_,
        maxAdvance_);

    if (!currentRoutePosition) {
        cancelRoute();
        return;
    }

    currentPath_ = path;
    routePositions_ = std::move(routePositions);
    currentRoutePosition_ = std::move(currentRoutePosition);

    onRouteChanged_(currentPath_);
    onRouteAdvanced_(currentRoutePosition_->advance, currentRoutePosition_->levelPoint);
}

std::optional<Router::RoutePosition> Router::getProjection(
    const std::vector<RoutePosition>& routePositions,
    const std::optional<RoutePosition>& currentRoutePosition,
    const std::optional<LevelPoint>& currentPosition,
    double maxProjectionDistance,
    double maxAdvance)
{
    if (!currentPosition || routePositions.empty()) {
        return std::nullopt;
    }

    const auto currentLevel = currentPosition->level;

    std::map<double, geometry::XYLineString> lines;
    bool lineFinished = true;

    auto addPoint = [&](const RoutePosition& pos) mutable {
        if (currentLevel != pos.levelPoint.level) {
            lineFinished = true;
            return;
        }

        if (lineFinished) {
            lines[pos.advance] = {};
        }

        lines.rbegin()->second.points.push_back(pos.levelPoint.point);
        lineFinished = false;
    };

    double startAdvance = 0;

    if (currentRoutePosition) {
        startAdvance = currentRoutePosition->advance;
        addPoint(*currentRoutePosition);
    } else {
        addPoint(routePositions.front());
    }

    double stopAdvance = startAdvance + maxAdvance;

    auto iter = std::upper_bound(
        routePositions.begin(),
        routePositions.end(),
        startAdvance,
        [](double advance, const auto& routePosition) {
            return advance < routePosition.advance;
        });

    for (; iter != routePositions.end(); ++iter) {
        assert(iter != routePositions.begin());
        auto prev = std::prev(iter);

        const auto& pBegin = *prev;
        const auto& pEnd = *iter;

        assert(pBegin.advance <= stopAdvance);

        if (pEnd.advance > stopAdvance) {
            if (pEnd.levelPoint.level == currentLevel) {
                const auto& pBeginXY = pBegin.levelPoint.point;

                const auto& pEndXY = pEnd.levelPoint.point;

                double k = (stopAdvance - pBegin.advance) /
                           (pEnd.advance - pBegin.advance);
                assert(0 <= k && k < 1);

                addPoint(RoutePosition{
                    .levelPoint =
                        LevelPoint{
                            .level = currentLevel,
                            .point = pBeginXY * (1 - k) + pEndXY * k},
                    .advance = stopAdvance,
                });
            }
            break;
        }

        addPoint(pEnd);
    }

    std::optional<geometry::Projection> bestProj;
    double bestAdvance = 0;
    for (const auto& [advance, line]: lines) {
        double projAdvance = 0;
        auto proj = GetProjection(line, currentPosition->point, &projAdvance);

        projAdvance += advance;
        if (!bestProj || bestProj->distance > proj.distance) {
            bestProj = proj;
            bestAdvance = projAdvance;
        }
    }

    if (!bestProj || bestProj->distance > maxProjectionDistance) {
        return std::nullopt;
    }

    return RoutePosition{
        .levelPoint =
            LevelPoint{
                .level = currentLevel,
                .point = bestProj->point},
        .advance = bestAdvance,
    };
}

} // namespace navigine::route_graph
