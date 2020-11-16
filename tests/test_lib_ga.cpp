#include <lib_ga.hpp>

#include "gtest/gtest.h"

TEST(MinCostMatching, matching)
{
  Problem P = Problem("../tests/instances/06.txt");

  LibGA::OpenList OPEN(LibGA::FieldEdge::compare);
  LibGA::FieldEdges GC_Edge;
  for (int i = 0; i < P.getNum(); ++i) {
    auto s = P.getStart(i);
    for (int j = 0; j < P.getNum(); ++j) {
      auto g = P.getGoal(j);
      auto p = new LibGA::FieldEdge(i, j, s, g, s->manhattanDist(g));
      p->setRealDist(P.getG()->pathDist(p->s, p->g));
      GC_Edge.push_back(p);
      OPEN.push(p);
    }
  }
  auto matching = LibGA::MinCostMatching(&P);
  while (!OPEN.empty()) {
    auto p = OPEN.top(); OPEN.pop();
    matching.addEdge(p);
    matching.update();
    if (matching.matched_num == P.getNum()) break;
  }
  for (auto p : GC_Edge) delete p;

  ASSERT_EQ(matching.assigned_goals[0]->id, 0);
  ASSERT_EQ(matching.assigned_goals[1]->id, 4);
  ASSERT_EQ(matching.assigned_goals[2]->id, 7);
}
