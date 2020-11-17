#pragma once
#include "solver.hpp"

class GoalSwapper : public Solver {
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

  Node* planOneStep(Agent* a,
                    std::unordered_map<Node*, Agent*>& occupied_now,
                    std::unordered_map<Node*, Agent*>& occupied_next);

  void run();

public:
  GoalSwapper(Problem* _P);
  ~GoalSwapper();

  static void printHelp();
};
