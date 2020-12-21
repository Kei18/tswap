cmake_minimum_required(VERSION 3.1)
project(unlabeled-mapf CXX)

add_subdirectory(./unlabeled_mapf)
add_subdirectory(./third_party/googletest)

add_executable(app ./app.cpp)
target_compile_features(app PUBLIC cxx_std_17)
target_link_libraries(app lib-unlabeled-mapf)

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ../unlabeled_mapf/include/*.hpp
  ../unlabeled_mapf/src/*.cpp
  ../tests/*.cpp
  ../app.cpp)

# for ILP
if(DEFINED ENV{GUROBI_HOME})
  add_definitions(-D_GUROBI_=1)
endif()

# test
set(TEST_MAIN_FUNC ./third_party/googletest/googletest/src/gtest_main.cc)
set(TEST_ALL_SRC ${TEST_MAIN_FUNC})
macro(add_test name target)
  add_executable(${name} ${target} ${TEST_MAIN_FUNC})
  target_link_libraries(${name} lib-unlabeled-mapf gtest)
  list(APPEND TEST_ALL_SRC ${target})
endmacro(add_test)

add_test(test_graph ./tests/test_graph.cpp)
add_test(test_problem ./tests/test_problem.cpp)
add_test(test_lib_ten ./tests/test_lib_ten.cpp)
add_test(test_ten ./tests/test_ten.cpp)
add_test(test_ten_incremental ./tests/test_ten_incremental.cpp)
add_test(test_network_flow ./tests/test_network_flow.cpp)
add_test(test_goal_allocator ./tests/test_goal_allocator.cpp)
add_test(test_naive_tswap ./tests/test_naive_tswap.cpp)
add_test(test_tswap ./tests/test_tswap.cpp)

add_executable(test ${TEST_ALL_SRC})
target_link_libraries(test lib-unlabeled-mapf gtest)
