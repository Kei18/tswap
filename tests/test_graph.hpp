#pragma once
#include "gtest/gtest.h"
#include <graph.hpp>

TEST(Graph, grid)
{
  Grid G("lak105d.map");
  ASSERT_TRUE(G.existNode(0));
  ASSERT_TRUE(G.existNode(455));
  ASSERT_TRUE(G.existNode(0, 0));
  ASSERT_FALSE(G.existNode(5));

  Node* v = G.getNode(0);
  Node* u = G.getNode(455);

  ASSERT_EQ(v->getDegree(), 2);
  ASSERT_EQ(v->manhattanDist(u), 35);
  ASSERT_TRUE(25 < v->euclideanDist(u) && v->euclideanDist(u) < 26);

  ASSERT_EQ(G.getV().size(), 443);
  ASSERT_EQ(G.getWidth(), 31);
  ASSERT_EQ(G.getHeight(), 25);
  ASSERT_EQ(G.getMapFileName(), "lak105d.map");
  ASSERT_EQ(G.pathDist(G.getNode(0), G.getNode(455)), 39);

  Config c1 = { v, u };
  Config c2 = { v, u };
  Config c3 = { u, v };
  ASSERT_TRUE(sameConfig(c1, c2));
  ASSERT_FALSE(sameConfig(c1, c3));
  ASSERT_TRUE(permutatedConfig(c1, c2));
  ASSERT_TRUE(permutatedConfig(c1, c3));
}
