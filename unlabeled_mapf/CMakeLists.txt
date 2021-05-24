cmake_minimum_required(VERSION 3.1)

file(GLOB SRCS "./src/*.cpp")
project(lib-unlabeled-mapf)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_definitions(-D_MAPDIR_="${CMAKE_CURRENT_LIST_DIR}/../map/")
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall -mtune=native -march=native)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE ./include)

add_subdirectory(../third_party/grid-pathfinding/graph ./graph)
target_link_libraries(lib-unlabeled-mapf lib-graph)

set(DUMMY_HEADER ./include/gurobi.hpp)
if(DEFINED ENV{GUROBI_HOME})
  message("use Gurobi (ILP Solver), generate header ${DUMMY_HEADER}")
  file(WRITE ${DUMMY_HEADER} "#include <gurobi_c++.h>")
  add_definitions(-D_GUROBI_=1)
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
  find_package(GUROBI REQUIRED)
  target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${GUROBI_LIB})
else()
  message("without GUROBI (ILP Solver), generate dummy header ${DUMMY_HEADER}")
  file(WRITE ${DUMMY_HEADER} "")
  file(APPEND ${DUMMY_HEADER} "using GRBEnv = int;
using GRBModel = int;
using GRBVar = int;
using GRBConstr = int;")
endif()
