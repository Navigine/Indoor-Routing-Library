#include <navigine/geometry/geometry.h>
#include <navigine/geometry/utils.h>
#include <navigine/route_graph/route_path.h>
#include <navigine/route_graph/utils.h>

namespace navigine::route_graph {

template<>
geometry::XYPoint GetPoint(const LevelPoint& levelPoint)
{
    return levelPoint.point;
}

namespace {

const double MIN_ROTATION_ANGLE = 10;       // degrees
const double MIN_ROTATION_RADIUS = 0.01;    // meters

double calculateRouteLength(const std::vector<LevelPoint>& points)
{
    if (points.size() < 2) {
        return 0;
    }

    double sum = 0;

    for(size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& pBeginXY = points[i].point;
        const auto& pEndXY = points[i + 1].point;

        const auto len = points[i].level == points[i + 1].level ?
            geometry::GetDist(pBeginXY, pEndXY) :
            0;

        sum += len;
    }
    return sum;
}

std::vector<LevelPoint> getSmoothPoly(
    const std::vector<LevelPoint>& points,
    double smoothRadius,
    std::vector<RouteEvent>* events)
{
    if (points.size() < 3) {
        return points;
    }

    // Smooth radius should be positive
    smoothRadius = std::max(smoothRadius, EPSILON);

    // Simplify polyline using Ramer–Douglas–Peucker algorithm:
    // https://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm
    const auto points1 = SimplifyPolyline(points, smoothRadius);
    assert(points1.size() >= 2);

    // Round polyline and calculate route events
    std::vector<LevelPoint> points2;
    points2.push_back(points1[0]);

    double dist = 0.0;
    for(size_t i = 1; i < points1.size() - 1; ++i) {
        const auto level = points1[i].level;
        const auto A = points1[i-1].point;
        const auto B = points1[i].point;
        const auto C = points1[i+1].point;

        const auto AB = geometry::GetDist(A, B);
        const auto BC = geometry::GetDist(B, C);

        // Calculating dot and cross products of vectors: BA, BC
        const auto dotProduct = (A.x - B.x) * (C.x - B.x) + (A.y - B.y) * (C.y - B.y);
        const auto crossProduct = (A.x - B.x) * (C.y - B.y) - (A.y - B.y) * (C.x - B.x);

        // Angle in triangle ABC using dot-product of BA and BC
        const auto angle = std::acos(std::clamp(dotProduct / AB / BC, -1.0, 1.0));
        const auto cosAngleByTwo = std::cos(angle / 2);
        const auto sinAngleByTwo = std::sin(angle / 2);

        // Calculating the length of the tangent segment
        auto len = smoothRadius * cosAngleByTwo / (1 - sinAngleByTwo);
        len = std::min(len, AB / 2);
        len = std::min(len, BC / 2);

        const auto k1 = len / AB;
        const auto k2 = len / BC;

        // Calculating tangent points on segments BA, BC
        const auto A1 = A * k1 + B * (1 - k1);
        const auto C1 = C * k2 + B * (1 - k2);

        // Calculating the rotation center O from equation:
        //  BO = (BA1 + BC1) / 2 / cosAngleByTwo^2
        const auto O = B + (A1 / 2 + C1 / 2 - B) / cosAngleByTwo / cosAngleByTwo;

        // Rotation radius (real smooth radius)
        const auto R = geometry::GetDist(O, A1);

        // Rotation event type
        const auto rotType = crossProduct > 0 ?
            RouteEvent::Type::TURN_RIGHT :
            RouteEvent::Type::TURN_LEFT;

        // Rotation event angle (in degrees)
        const auto rotAngle = std::round(geometry::RadToDeg(M_PI - angle));

        if (R < MIN_ROTATION_RADIUS) {
            // Not smoothing angle
            const auto prevPoint = points2.back();
            const auto currPoint = LevelPoint{
                .level = level,
                .point = B};

            assert(prevPoint.level == currPoint.level);

            points2.push_back(currPoint);
            dist += geometry::GetDist(prevPoint.point, currPoint.point);

            if (rotAngle >= MIN_ROTATION_ANGLE && events) {
                events->push_back(RouteEvent{
                    .type = rotType,
                    .value = static_cast<int>(rotAngle),
                    .distance = dist});
            }
        } else {
            // Smoothing angle
            const auto alpha1 = std::atan2(A1.y - O.y, A1.x - O.x); // Radius-vector (O,A1) direction
            const auto alpha2 = std::atan2(C1.y - O.y, C1.x - O.x); // Radius-vector (O,C1) direction
            const auto delta = geometry::NormalizedAngle(alpha2 - alpha1);

            int nsteps = std::round(geometry::RadToDeg(std::fabs(delta)) / MIN_ROTATION_ANGLE);

            // Number of steps should be even
            nsteps = std::max(nsteps, 2);
            if (nsteps % 2 == 1) {
                nsteps++;
            }

            for(int k = 0; k <= nsteps; ++k) {
                const auto alpha = geometry::NormalizedAngle(alpha1 + k * delta / nsteps);
                const auto prevPoint = points2.back();
                const auto currPoint = LevelPoint{
                    .level = level,
                    .point = geometry::XYPoint{
                        .x = O.x + R * std::cos(alpha),
                        .y = O.y + R * std::sin(alpha)}};

                assert(prevPoint.level == currPoint.level);

                points2.push_back(currPoint);
                dist += geometry::GetDist(prevPoint.point, currPoint.point);

                if (rotAngle >= MIN_ROTATION_ANGLE && events && k == nsteps / 2) {
                    events->push_back(RouteEvent{
                        .type = rotType,
                        .value = static_cast<int>(rotAngle),
                        .distance = dist});
                }
            }
        }
    }
    points2.push_back(points1.back());
    return points2;
}

} // namespace

RoutePath::RoutePath(
    const std::vector<LevelPoint>& points,
    const std::vector<RouteEvent>& events)
    : points_(points)
    , events_(events)
    , length_(calculateRouteLength(points))
{}

double RoutePath::length() const
{
    return length_;
}

const std::vector<LevelPoint>& RoutePath::points() const
{
    return points_;
}

const std::vector<RouteEvent>& RoutePath::events() const
{
    return events_;
}

std::pair<SharedRoutePath, SharedRoutePath> RoutePath::split(double advance) const
{
    if (points_.empty()) {
        return {nullptr, nullptr};
    }

    if (advance < 0) {
        return {nullptr, std::make_shared<RoutePath>(*this)};
    }

    if (advance > length_) {
        return {std::make_shared<RoutePath>(*this), nullptr};
    }

    std::vector<LevelPoint> leftPoints;
    std::vector<LevelPoint> rightPoints;

    leftPoints.push_back(points_.front());

    double totalLength = 0;
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
        const auto& pBeginXY = points_[i].point;
        const auto& pEndXY = points_[i + 1].point;

        const auto len = points_[i].level == points_[i + 1].level ?
            geometry::GetDist(pBeginXY, pEndXY) :
            0;

        if (totalLength + len <= advance) {
            leftPoints.push_back(points_[i + 1]);
        }

        if (totalLength >= advance) {
            rightPoints.push_back(points_[i]);
        }

        if (advance - len < totalLength && totalLength < advance) {
            assert(len > 0);
            assert(points_[i].level == points_[i + 1].level);

            double k = (advance - totalLength) / len;

            assert(0 < k && k < 1);
            const auto pMidXY = pBeginXY * (1 - k) + pEndXY * k;

            leftPoints.push_back(LevelPoint{
                .level = points_[i].level,
                .point = pMidXY});

            rightPoints.push_back(LevelPoint{
                .level = points_[i].level,
                .point = pMidXY});
        }

        totalLength += len;
    }

    rightPoints.push_back(points_.back());

    std::vector<RouteEvent> leftEvents;
    std::vector<RouteEvent> rightEvents;

    for (const auto& event : events_) {
        if (event.distance < advance) {
            leftEvents.push_back(event);
        }

        if (event.distance > advance) {
            auto rightEvent = event;
            rightEvent.distance -= advance;
            rightEvents.push_back(rightEvent);
        }

        if (event.distance == advance) {
            if (event.type == RouteEvent::Type::TRANSITION) {
                leftEvents.push_back(event);

                auto rightEvent = event;
                rightEvent.distance -= advance;
                rightEvents.push_back(rightEvent);
            }
        }
    }

    return {
        std::make_shared<RoutePath>(leftPoints, leftEvents),
        std::make_shared<RoutePath>(rightPoints, rightEvents)};
}

SharedRoutePath RoutePath::createSmoothPath(const std::vector<LevelPoint>& points, double smoothRadius)
{
    if (points.empty()) {
        return nullptr;
    }

    // RoutePath path;
    std::vector<LevelPoint> resPoints;
    std::vector<RouteEvent> resEvents;
    double resDistance = 0;
    for(size_t i = 0, i0 = 0; i0 < points.size(); i0 = i) {
        while (i < points.size() && points[i].level == points[i0].level) {
            ++i;
        }

        if (!resPoints.empty()) {
            resEvents.push_back(RouteEvent{
                .type = RouteEvent::Type::TRANSITION,
                .value = static_cast<int>(points[i0].level),
                .distance = resDistance});
        }

        std::vector<RouteEvent> levelEvents;

        const auto levelPoints = getSmoothPoly(
            std::vector<LevelPoint>(points.begin() + i0, points.begin() + i),
            smoothRadius,
            &levelEvents);

        for(size_t j = 0; j < levelEvents.size(); ++j) {
            resEvents.push_back(RouteEvent{
                .type = levelEvents[j].type,
                .value = levelEvents[j].value,
                .distance = levelEvents[j].distance + resDistance});
        }

        resPoints.push_back(levelPoints[0]);
        for(size_t j = 1; j < levelPoints.size(); ++j) {
            assert(levelPoints[j].level == levelPoints[j-1].level);
            resDistance += geometry::GetDist(levelPoints[j].point, levelPoints[j-1].point);
            resPoints.push_back(levelPoints[j]);
        }
    }
    return std::make_shared<RoutePath>(std::move(resPoints), std::move(resEvents));
}

std::ostream& operator<<(std::ostream& out, const RouteEvent& e)
{
    std::string typeStr;
    switch (e.type) {
        case RouteEvent::Type::TURN_LEFT:
            typeStr = "TURN_LEFT";
            break;
        case RouteEvent::Type::TURN_RIGHT:
            typeStr = "TURN_RIGHT";
            break;
        case RouteEvent::Type::TRANSITION:
            typeStr = "TRANSITION";
            break;
    }
    out << "RouteEvent[" << typeStr << ", value=" << e.value << ", distance=" << e.distance << "]";
    return out;
}

std::ostream& operator<<(std::ostream& out, const RoutePath& routePath)
{
    out << "Total length:" << routePath.length_ << std::endl;
    out << "Points:" << std::endl;
    for (const auto& lp : routePath.points_) {
        out << "  " << lp << std::endl;
    }

    out << "Events:" << std::endl;
    for (const auto& e : routePath.events_) {
        out << "  " << e << std::endl;
    }

    return out;
}

} // namespace navigine::route_graph
