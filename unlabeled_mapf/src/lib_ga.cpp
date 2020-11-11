#include "../include/lib_ga.hpp"

LibGA::FieldEdge::FieldEdge(int sindex, int gindex, Node* _s, Node* _g, int _d)
  : start_index(sindex),
    goal_index(gindex),
    s(_s),
    g(_g),
    evaled(false),
    inst_d(_d),
    d(0)
{
}

void LibGA::FieldEdge::setRealDist(int _d)
{
  if (!evaled) {
    evaled = true;
    d = _d;
  }
}

bool LibGA::FieldEdge::compare(FieldEdge* a, FieldEdge* b)
{
  if (a->evaled && b->evaled) {
    if (a->d != b->d) return a->d > b->d;
  } else if (!a->evaled && !b->evaled) {
    if (a->inst_d != b->inst_d) return a->inst_d > b->inst_d;
  } else if (a->evaled && !b->evaled) {
    if (a->d != b->inst_d) return a->d > b->inst_d;
  } else if (!a->evaled && b->evaled) {
    if (a->inst_d != b->d) return a->inst_d > b->d;
  }
  // tie break
  if (a->start_index != b->start_index) return a->start_index < b->start_index;
  return a->g->id < b->g->id;
}
