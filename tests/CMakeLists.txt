enable_testing()

add_executable(run_test
    main.cpp
    graph_test.cpp
    route_graph_test.cpp
    router_test.cpp
    split_test.cpp
    scc_test.cpp
    simplify_test.cpp
    route_graph_utils_test.cpp
)

add_test(NAME test_route_graph_1_5 COMMAND run_test test_route_graph_1_5)
add_test(NAME test_route_graph_1_4 COMMAND run_test test_route_graph_1_4)

add_test(NAME test_route_graph_get_path COMMAND run_test test_route_graph_get_path)
add_test(NAME test_route_graph_empty_graph COMMAND run_test test_route_graph_empty_graph)
add_test(NAME test_route_graph_different_level_graph COMMAND run_test test_route_graph_different_level_graph)
add_test(NAME test_route_graph_get_paths COMMAND run_test test_route_graph_get_paths)

add_test(NAME test_router_basic_test_case COMMAND run_test test_router_basic_test_case)
add_test(NAME test_router_basic_test_path1 COMMAND run_test test_router_basic_test_path1)
add_test(NAME test_router_basic_test_path2 COMMAND run_test test_router_basic_test_path2)
add_test(NAME test_router_basic_test_path3 COMMAND run_test test_router_basic_test_path3)

add_test(NAME test_split_case_1 COMMAND run_test test_split_case_1)
add_test(NAME test_split_case_2 COMMAND run_test test_split_case_2)
add_test(NAME test_split_case_3 COMMAND run_test test_split_case_3)

add_test(NAME test_subgraph COMMAND run_test test_subgraph)
add_test(NAME test_scc COMMAND run_test test_scc)

add_test(NAME test_simplify_cross COMMAND run_test test_simplify_cross)
add_test(NAME test_triangle_1 COMMAND run_test test_triangle_1)
add_test(NAME test_triangle_2 COMMAND run_test test_triangle_2)
add_test(NAME test_triangle_3 COMMAND run_test test_triangle_3)

add_test(NAME test_route_graph_utils_case_1 COMMAND run_test test_route_graph_utils_case_1)
add_test(NAME test_route_graph_utils_case_2 COMMAND run_test test_route_graph_utils_case_2)
add_test(NAME test_route_graph_utils_case_3 COMMAND run_test test_route_graph_utils_case_3)
add_test(NAME test_route_graph_utils_case_4 COMMAND run_test test_route_graph_utils_case_4)
add_test(NAME test_route_graph_utils_case_5 COMMAND run_test test_route_graph_utils_case_5)

target_link_libraries(run_test route_graph)
