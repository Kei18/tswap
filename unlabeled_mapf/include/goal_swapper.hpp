#pragma once
#include "solver.hpp"

class GoalSwapper : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  struct Agent {
    int id;  // id
    Node* v_now;
    Node* v_next;
    Node* g;  // goal location
    int called;
  };

  bool use_bfs_allocate;

  // for log
  int elapsed_assignment;
  int elapsed_pathplanning;
  int estimated_makespan;
  int estimated_soc;

  Node* planOneStep(Agent* a, std::unordered_map<Node*, Agent*>& occupied_now,
                    std::unordered_map<Node*, Agent*>& occupied_next);
  bool deadlockDetectResolve(Agent* a,
                             std::unordered_map<Node*, Agent*>& occupied_now);
  void run();

public:
  GoalSwapper(Problem* _P);
  ~GoalSwapper();

  void setParams(int argc, char* argv[]);
  static void printHelp();

  void makeLog(const std::string& logfile);
};
