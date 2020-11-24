#pragma once
#include "lib_ten.hpp"
#include "plan.hpp"
#include "problem.hpp"

class TEN
{
protected:
  Problem* const P;  // original problem
  const Nodes V;     // all nodes in G

  LibTEN::ResidualNetwork network;
  bool valid_network;
  Plan solution;

private:
  const int max_timestep;  // max timestep

protected:
  void extendGraphOneTimestep(const int t);
  virtual void updateGraph();
  void createPlan(const int T);
  virtual void createPlan();

public:
  TEN(Problem* const _P, const int _T, const bool _apply_filter = false,
      const bool _use_ilp_solver = false);
  virtual ~TEN();

  virtual void update();
  virtual void update(const int t);

  bool isValid() { return valid_network; }
  Plan getPlan() { return solution; }

  void resetFlow() { network.clearAllCapacity(); }

  void setTimeLimit(int _time_limit) { network.setTimeLimit(_time_limit); }

  int getNodesNum();
  int getEdgesNum();

  // for Ford Furlkerson
  int getDfsCnt();

#ifdef _GUROBI_
  int getVariantsCnt();
  int getConstraintsCnt();
#endif
};
