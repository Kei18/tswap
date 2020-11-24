#pragma once
#include <memory>

#include "../include/ten.hpp"
#include "../include/ten_incremental.hpp"
#include "solver.hpp"

class NetworkFlow : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool use_incremental;   // choose TEN_INCREMENTAL or TEN
  bool use_filter;        // apply filter or not
  bool use_minimum_step;  // use minimum step
  bool use_ilp_solver;    // use ILP solver
  bool use_binary_search;
  bool use_past_flow;
  int minimum_step;

  void setupMinimumStep();
  void printAdditionalInfo(int t, std::shared_ptr<TEN> flow_network);
  void binaryRun();

  void run();

public:
  NetworkFlow(Problem* _P);
  ~NetworkFlow();

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
