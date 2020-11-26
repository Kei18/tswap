#pragma once
#include "ten.hpp"

class TEN_INCREMENTAL : public TEN
{
private:
  int current_timestep;

  void updateGraph();
  void createPlan();

public:
  TEN_INCREMENTAL(Problem* const _P, const bool _filter = false,
                  const bool _ilp = false);
  TEN_INCREMENTAL(Problem* const _P, const int _t, const bool _filter = false,
                  const bool _ilp = false, int _time_limit = -1);
  ~TEN_INCREMENTAL();

  void update();
  void update(const int t);
};
