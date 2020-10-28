#pragma once
#include "ten.hpp"

class TEN_INCREMENTAL : public TEN {
private:
  int current_timestep;

  void updateGraph();
  void createPlan();

public:
  TEN_INCREMENTAL(Problem* const _P);
  ~TEN_INCREMENTAL();

  void update();
};
