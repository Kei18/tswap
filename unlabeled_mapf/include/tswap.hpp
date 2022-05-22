/*
 * TSWAP, used in the experiment
 */

#pragma once
#include "../include/goal_allocator.hpp"
#include "solver.hpp"

class TSWAP : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  struct Agent {
    int id;        // id
    Node* v_now;   // current location
    Node* v_next;  // next location
    Node* g;       // goal location
    int called;    // how many times called in the queue
  };

  GoalAllocator::MODE assignment_mode;
  std::shared_ptr<GoalAllocator> allocator;  // target assignment algorithm
  std::vector<int> goal_indexes;  // node-id -> goal index \in {1, ..., N}}, used with lazy distance evaluation

  // for log
  int elapsed_assignment;    // elapsed time for target assignment
  int elapsed_pathplanning;  // elapsed time for path planing
  int estimated_makespan;    // estimated makespan according to the target
                             // assignment
  int estimated_soc;         // estimated sum-of-costs according to the target
                             // assignment

  Node* getNextNode(Node* a, Node* b);

  // detect and resolve deadlocks
  bool deadlockDetectResolve(Agent* a, std::vector<Agent*>& occupied_now);

  void run();

public:
  TSWAP(Problem* _P);
  ~TSWAP();

  void setParams(int argc, char* argv[]);
  static void printHelp();

  void makeLog(const std::string& logfile);
};
