#pragma once


#include <navigine/route_graph/route_graph.h>
#include <navigine/geometry/utils.h>

namespace navigine::route_graph::test_utils {

inline bool testCheckFull(const RouteGraph& routeGraph)
{
    std::vector<std::string> errors;
    const auto errCount = routeGraph.checkFull(&errors);

    for (const auto& error : errors) {
        std::cout << "Error: " << error << "\n";
    }

    return !errCount;
}

inline bool testPointsCount(unsigned int curSize, unsigned int expSize)
{
    if (curSize != expSize) {
        std::cout << "Invalid points size: " << curSize << " expected: " << expSize << "\n";
        return false;
    }
    return true;
}

inline bool testCloseValues(double curValue, double expValue)
{
    if (!geometry::checkCloseValues(curValue, expValue)) {
        std::cout << "Invalid close values rurrent: " << curValue << " expected: " << expValue << " diff: " << std::fabs(curValue - expValue) << "\n";
        return false;
    }
    return true;
}

template <typename T>
inline bool testEqualValues(T curValue, T expValue)
{
    if (curValue != expValue) {
        std::cout << "Invalid equal values rurrent: " << curValue << " expected: " << expValue << "\n";
        return false;
    }
    return true;
}

} // namespace navigine::route_graph::test_utils