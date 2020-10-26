cmake_minimum_required(VERSION 3.1)
project(unlabeled-mapf CXX)
add_subdirectory(./mapf)
add_executable(app ./mapf/src/main.cpp)
target_compile_features(app PUBLIC cxx_std_17)
target_link_libraries(app lib-mapf)

find_package(GTest REQUIRED)
add_executable(test ./test/test.cpp)
target_compile_features(test PUBLIC cxx_std_17)
target_link_libraries(test
  lib-mapf
  GTest::GTest
  GTest::Main)
