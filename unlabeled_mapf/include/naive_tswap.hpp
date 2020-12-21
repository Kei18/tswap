#pragma once
#include "solver.hpp"

class NaiveTSWAP : public Solver
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

  // for log
  int elapsed_assignment;
  int elapsed_pathplanning;
  int estimated_makespan;
  int estimated_soc;

  void run();

public:
  NaiveTSWAP(Problem* _P);
  ~NaiveTSWAP();

  void setParams(int argc, char* argv[]);
  static void printHelp();

  void makeLog(const std::string& logfile);
};
