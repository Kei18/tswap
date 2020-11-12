cmake_minimum_required(VERSION 3.1)

file(GLOB SRCS "./src/*.cpp")
project(lib-unlabeled-mapf)
add_library(${PROJECT_NAME} STATIC ${SRCS})
add_definitions(-D_MAPDIR_="${CMAKE_CURRENT_LIST_DIR}/../map/")
target_compile_options(${PROJECT_NAME} PUBLIC -O3 -Wall -mtune=native -march=native)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE ./include)

cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
message(${HOST})

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(GUROBI REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${GUROBI_LIB})
