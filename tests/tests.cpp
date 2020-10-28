#include "gtest/gtest.h"
#include "test_graph.hpp"
#include "test_problem.hpp"
#include "test_lib_ten.hpp"
#include "test_ten.hpp"
#include "test_ten_incremental.hpp"
#include "test_network_flow.hpp"

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
