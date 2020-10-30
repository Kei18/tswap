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
  TEN(Problem* const _P, const int _T, const bool _apply_filter=false);
  virtual ~TEN();

  virtual void update();

  bool isValid() { return valid_network; }
  Plan getPlan() { return solution; }

  int getNodesNum();
  int getEdgesNum();
  int getDfsCnt();
};
