add_library(route_graph
    graph.cpp
    route_graph.cpp
    route_path.cpp
    router.cpp
    route_graph_utils.cpp
)

add_library(route_graph::route_graph ALIAS route_graph)
