#include <iostream>
#include <iomanip>
#include <cstring>

#define TERMINAL_RED        "\033[91m"
#define TERMINAL_DEFAULT    "\033[39m"

#define ADD_TEST(test_name)                                     \
    do {                                                        \
        extern int test_name();                                 \
        if (argc >= 2 && !std::strcmp(argv[1], #test_name)) {   \
            std::cout << TERMINAL_RED;                          \
            int ret = test_name();                              \
            std::cout << TERMINAL_DEFAULT;                      \
            return ret;                                         \
        }                                                       \
    } while (false)


int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " test_name" << std::endl;
        return -1;
    }

    std::cout << std::setprecision(20);

    ADD_TEST(test_route_graph_1_5);
    ADD_TEST(test_route_graph_1_4);


    ADD_TEST(test_route_graph_get_path);
    ADD_TEST(test_route_graph_empty_graph);
    ADD_TEST(test_route_graph_different_level_graph);
    ADD_TEST(test_route_graph_get_paths);

    ADD_TEST(test_router_basic_test_case);
    ADD_TEST(test_router_basic_test_path1);
    ADD_TEST(test_router_basic_test_path2);
    ADD_TEST(test_router_basic_test_path3);

    ADD_TEST(test_split_case_1);
    ADD_TEST(test_split_case_2);
    ADD_TEST(test_split_case_3);

    ADD_TEST(test_subgraph);
    ADD_TEST(test_scc);

    ADD_TEST(test_simplify_cross);
    ADD_TEST(test_triangle_1);
    ADD_TEST(test_triangle_2);
    ADD_TEST(test_triangle_3);

    ADD_TEST(test_route_graph_utils_case_1);
    ADD_TEST(test_route_graph_utils_case_2);
    ADD_TEST(test_route_graph_utils_case_3);
    ADD_TEST(test_route_graph_utils_case_4);
    ADD_TEST(test_route_graph_utils_case_5);

    std::cout << TERMINAL_RED;
    std::cout << "Test not found: " << argv[1] << std::endl;
    std::cout << TERMINAL_DEFAULT;
    return -1;
}
