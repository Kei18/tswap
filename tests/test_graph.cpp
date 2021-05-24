#include <graph.hpp>
#include <problem.hpp>

#include "gtest/gtest.h"

TEST(Graph, loading)
{
  Grid G("lak105d.map");
  ASSERT_TRUE(G.existNode(0));
  ASSERT_TRUE(G.existNode(455));
  ASSERT_TRUE(G.existNode(0, 0));
  ASSERT_FALSE(G.existNode(5));
  ASSERT_EQ(G.getMapFileName(), "lak105d.map");
  ASSERT_EQ(G.getNode(0)->getDegree(), 2);
  ASSERT_EQ(G.getV().size(), 443);
  ASSERT_EQ(G.getWidth(), 31);
  ASSERT_EQ(G.getHeight(), 25);
}

TEST(Graph, config)
{
  Grid G("8x8.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(1);

  Config c1 = {v, u};
  Config c2 = {v, u};
  Config c3 = {u, v};
  ASSERT_TRUE(sameConfig(c1, c2));
  ASSERT_FALSE(sameConfig(c1, c3));
  ASSERT_TRUE(permutatedConfig(c1, c2));
  ASSERT_TRUE(permutatedConfig(c1, c3));
}

TEST(Graph, distance)
{
  Grid G("lak105d.map");
  Node* v = G.getNode(0);
  Node* u = G.getNode(455);
  ASSERT_EQ(v->manhattanDist(u), 35);
  ASSERT_TRUE(25 < v->euclideanDist(u) && v->euclideanDist(u) < 26);
  ASSERT_EQ(G.pathDist(v, u), 39);
}

TEST(Graph, large_field)
{
  Grid G("brc202d.map");
  Node* v = G.getNode(216, 138);
  Node* u = G.getNode(203, 303);
  ASSERT_EQ(G.pathDist(v, u), 782);
}

TEST(Graph, huge_field)
{
  Grid G("ost000a.map");
  Node* v = G.getNode(273, 721);
  Node* u = G.getNode(84, 461);
  ASSERT_EQ(G.pathDist(v, u), 863);
}

TEST(Graph, bfs)
{
  Grid G("lak105d.map");
  Node* s = G.getNode(0);
  Node* g = G.getNode(455);
  G.BFS(s);
  ASSERT_TRUE(G.isPathComputed(s, g));
  ASSERT_EQ(G.pathDist(s, g), 39);

  Node* v = G.getNode(44);
  Node* u1 = G.getNode(75);
  Node* u2 = G.getNode(168);
  Node* u3 = G.getNode(722);
  Nodes goals = {u1, u2};
  G.BFS(v, goals);
  ASSERT_TRUE(G.isPathComputed(v, u1));
  ASSERT_TRUE(G.isPathComputed(v, u2));
  ASSERT_FALSE(G.isPathComputed(v, u3));
}

TEST(Graph, bfs_huge)
{
  Grid G("ost000a.map");
  Node* v = G.getNode(273, 721);
  Node* u = G.getNode(84, 461);
  Nodes goals = {u};
  G.BFS(v, goals);
  ASSERT_TRUE(G.isPathComputed(v, u));
  ASSERT_EQ(G.pathDist(v, u), 863);
}
