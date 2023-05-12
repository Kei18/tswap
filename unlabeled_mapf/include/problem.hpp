#pragma once
#include <graph.hpp>
#include <random>
#include <set>

#include "default_params.hpp"
#include "util.hpp"

using Config = std::vector<Node*>;  // < loc_0[t], loc_1[t], ... >
using Configs = std::vector<Config>;

[[maybe_unused]] static bool sameConfig(const Config& config_i,
                                        const Config& config_j)
{
  if (config_i.size() != config_j.size()) return false;
  const int size_i = config_i.size();
  for (int k = 0; k < size_i; ++k) {
    if (config_i[k] != config_j[k]) return false;
  }
  return true;
}

[[maybe_unused]] static bool permutatedConfig(const Config& config_i,
                                              const Config& config_j)
{
  std::set<Node*> set_config_i(config_i.begin(), config_i.end());
  std::set<Node*> set_config_j(config_j.begin(), config_j.end());

  return set_config_i == set_config_j;
}

class Problem
{
private:
  std::string instance;             // instance name
  Graph* G;                         // graph
  std::mt19937* MT;                 // seed
  Config config_s;                  // initial configuration
  Config config_g;                  // goal configuration
  int num_agents;                   // number of agents
  int max_timestep;                 // timestep limit
  int max_comp_time;                // comp_time limit, ms

  const bool instance_initialized;  // for memory manage

  enum ScenarioType { USER_SPECIFIED, RANDOM };

  // set starts and goals randomly
  void setRandomStartsGoals(const int flocking_blocks = 0);

public:
  Problem(const std::string& _instance);
  Problem(Problem* P, Config _config_s, Config _config_g, int _max_comp_time,
          int _max_timestep);
  ~Problem();

  Graph* getG() { return G; }
  int getNum() { return num_agents; }
  std::mt19937* getMT() { return MT; }
  Node* getStart(int i) const;  // return start of a_i
  Node* getGoal(int i) const;   // return  goal of a_i
  Config getConfigStart() const { return config_s; };
  Config getConfigGoal() const { return config_g; };
  int getMaxTimestep() { return max_timestep; };
  int getMaxCompTime() { return max_comp_time; };
  std::string getInstanceFileName() { return instance; };

  // used when making new instance file
  void makeScenFile(const std::string& output_file);
};
