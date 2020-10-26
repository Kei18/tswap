#pragma once
#include "solver.hpp"

class NetworkBasedSolver : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  void run();

public:
  NetworkBasedSolver(Problem* _P);
  ~NetworkBasedSolver();

  static void printHelp();
};
