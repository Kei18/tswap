/*
 * Implementation of the makespan optimal algorithm
 *
 * - ref
 * Yu, J., & LaValle, S. M. (2013).
 * Multi-agent path planning and network flow.
 * In Algorithmic foundations of robotics X (pp. 157-173). Springer, Berlin,
 * Heidelberg.
 */

#pragma once
#include <memory>

#include "../include/ten.hpp"
#include "../include/ten_incremental.hpp"
#include "solver.hpp"

class FlowNetwork : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  bool use_aggressive_lower_bound;  // LB: use minimum step or not, default:
                                    // false
  bool use_passive_lower_bound;     // use real distance to compute LB, default:
                                    // false
  bool use_binary_search;           // Binary: use binary search, default: false
  bool use_pruning;                 // Prune: pruning redundant vertices
  bool use_past_flow;               // Reuse: use past flow, default: true

  bool use_incremental;  // choose TEN_INCREMENTAL or TEN (no cache), default:
                         // true

  int minimum_step;  // start from this timestep
  bool is_optimal;   // for binary search, optimal makespan or not

  void run();

  // for log
  struct HIST {
    int elapsed;          // elapsed time
    int makespan;         // makespan limit
    bool valid;           // valid unlabeled-MAPF result
    int visited_nodes;    // the number of nodes visited in the Ford-Fulkerson
                          // algorithm
    int network_size;     // network size
    int variants_cnt;     // for ILP, the number of variables
    int constraints_cnt;  // for ILP, the number of constraints
  };
  std::vector<HIST> HISTS;

public:
  FlowNetwork(Problem* _P);
  ~FlowNetwork();

  void setParams(int argc, char* argv[]);
  static void printHelp();

  void makeLog(const std::string& logfile);
};
