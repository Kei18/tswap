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
  bool use_incremental;   // choose TEN_INCREMENTAL or TEN, default: true
  bool use_filter;        // apply filter or not, default: true
  bool use_minimum_step;  // use minimum step or not, default: false
  bool use_ilp_solver;    // use ILP solver or not, default: false
  bool use_binary_search; // use binary search, default: false
  bool use_past_flow;     // use past flow, default: true
  int minimum_step;

  void run();

  // for log
  struct HIST {
    int elapsed;
    int makespan;
    bool valid;
    int visited_nodes;
    int network_size;
    int variants_cnt;
    int constraints_cnt;
  };
  std::vector<HIST> HISTS;
  bool is_optimal;

public:
  NetworkFlow(Problem* _P);
  ~NetworkFlow();

  void setParams(int argc, char* argv[]);
  static void printHelp();

  void makeLog(const std::string& logfile);
};
