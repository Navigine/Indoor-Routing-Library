cmake_minimum_required(VERSION 3.13)

project(route_graph VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
option(BUILD_ROUTE_GRAPH_TESTS "If enabled we will compile unit tests." OFF)

include(CTest)

add_subdirectory(src)

add_subdirectory(geometry)

target_include_directories(route_graph PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
)

target_link_libraries(route_graph PUBLIC geometry::geometry)

if(BUILD_TESTS OR BUILD_ROUTE_GRAPH_TESTS)
    add_subdirectory(tests)
    enable_testing()
endif()
