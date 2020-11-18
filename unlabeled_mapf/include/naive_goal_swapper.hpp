#pragma once
#include "solver.hpp"

class NaiveGoalSwapper : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  struct Agent {
    int id;
    Node* v;
    Node* g;
  };

  void run();

public:
  NaiveGoalSwapper(Problem* _P);
  ~NaiveGoalSwapper();

  static void printHelp();
};
