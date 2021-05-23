/*
 * Time Expanded Network, TEN
 */

#pragma once
#include "lib_ten.hpp"
#include "plan.hpp"
#include "problem.hpp"

class TEN
{
protected:
  Problem* const P;  // original problem
  const Nodes V;     // all nodes in G

  LibTEN::ResidualNetwork network;  // main body
  bool valid_network;               //
  Plan solution;                    // generate solutions
  int time_limit;                   // time limit
  Time::time_point t_start;         // to measure computation time

private:
  const int max_timestep;  // max timestep

protected:
  // extend time expanded network only for one timestep
  void extendGraphOneTimestep(const int t);

  // create time expanded network
  virtual void updateGraph();

  // convert the flow to the plan of unlabeled-MAPF
  void createPlan(const int T);
  virtual void createPlan();

  // check time limit
  bool overCompTime() const;

public:
  TEN(Problem* const _P, const int _T, const bool _apply_filter = false);
  virtual ~TEN();

  // update time expanded network
  virtual void update();
  virtual void update(const int t);

  bool isValid() { return valid_network; }
  Plan getPlan() { return solution; }

  // clear all flows
  void resetFlow() { network.clearAllCapacity(); }

  // set time limit of computation time
  void setTimeLimit(int _time_limit);

  // return info of time expanded network
  int getNodesNum();
  int getEdgesNum();

  // return the number of visited nodes by the Ford Furlkerson algorithm
  int getDfsCnt();

  // for ILP
#ifdef _GUROBI_
  // return the number of variants or constraints
  int getVariantsCnt();
  int getConstraintsCnt();
#endif
};
