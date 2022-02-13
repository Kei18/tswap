/*
 * TSWAP without modifications from the pseudo-code in the paper.
 */

#pragma once
#include "solver.hpp"

class NaiveTSWAP : public Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  struct Agent {
    int id;
    Node* v;  // current location
    Node* g;  // current target
  };

  // for log
  int elapsed_assignment;    // elapsed time for target assignment
  int elapsed_pathplanning;  // elapsed time for path planing
  int estimated_makespan;    // estimated makespan according to the target
                             // assignment
  int estimated_soc;         // estimated sum-of-costs according to the target
                             // assignment

  void run();

public:
  NaiveTSWAP(Problem* _P);
  ~NaiveTSWAP();

  void setParams(int argc, char* argv[]);
  static void printHelp();

  void makeLog(const std::string& logfile);
};
