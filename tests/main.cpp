#include "gtest/gtest.h"
#include "graph.hpp"
#include "problem.hpp"
#include "network_flow.hpp"
#include "incremental_ten.hpp"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
