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

  bool use_bfs_allocate;

  void run();

public:
  NaiveGoalSwapper(Problem* _P);
  ~NaiveGoalSwapper();

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
