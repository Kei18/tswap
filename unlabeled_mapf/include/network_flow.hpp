#pragma once
#include "solver.hpp"

class NetworkFlow : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  void run();

public:
  NetworkFlow(Problem* _P);
  ~NetworkFlow();

  static void printHelp();
};
