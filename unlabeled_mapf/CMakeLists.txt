cmake_minimum_required(VERSION 3.1)

file(GLOB SRCS "./src/*.cpp")
project(lib-unlabeled-mapf)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_definitions(-D_MAPDIR_="${CMAKE_CURRENT_LIST_DIR}/../map/")
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE ./include)

add_subdirectory(../third_party/grid-pathfinding/graph ./graph)
target_link_libraries(lib-unlabeled-mapf lib-graph)
