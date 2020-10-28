cmake_minimum_required(VERSION 3.1)
project(unlabeled-mapf CXX)

add_subdirectory(./unlabeled_mapf)
add_subdirectory(./third_party/googletest)

add_executable(app ./app.cpp)
target_compile_features(app PUBLIC cxx_std_17)
target_link_libraries(app lib-unlabeled-mapf)

add_executable(test ./tests/tests.cpp)
target_compile_features(test PUBLIC cxx_std_17)
target_include_directories(test PUBLIC ./third_party/googletest/googletest/include)
target_link_libraries(test lib-unlabeled-mapf gtest)
