#pragma once
#include "solver.hpp"

class NetworkFlow : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool use_incremental;  // choose TEN_INCREMENTAL or TEN (Time Expanded Network)

  void run();

public:
  NetworkFlow(Problem* _P);
  ~NetworkFlow();

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
