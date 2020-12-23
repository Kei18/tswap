#include "../include/problem.hpp"

#include <fstream>
#include <regex>

#include "../include/util.hpp"

Problem::Problem(const std::string& _instance)
    : instance(_instance), instance_initialized(true)
{
  // read instance file
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  ScenarioType scen_type = ScenarioType::USER_SPECIFIED;
  int flocking_blocks = 0;

  std::string line;
  std::smatch results;
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_random_problem = std::regex(R"(random_problem=(\d+))");
  std::regex r_flocking_blocks = std::regex(R"(flocking_blocks=(\d+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+),(\d+),(\d+))");

  while (getline(file, line)) {
    // comment
    if (std::regex_match(line, results, r_comment)) {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      num_agents = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed)) {
      MT = new std::mt19937(std::stoi(results[1].str()));
      continue;
    }
    // skip reading initial/goal nodes
    if (std::regex_match(line, results, r_random_problem)) {
      int val = std::stoi(results[1].str());
      if (val == (int)ScenarioType::USER_SPECIFIED) {
        scen_type = ScenarioType::USER_SPECIFIED;
      } else {
        scen_type = ScenarioType::RANDOM;
      }
      continue;
    }
    // set flocking blocks
    if (std::regex_match(line, results, r_flocking_blocks)) {
      flocking_blocks = std::stoi(results[1].str());
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep)) {
      max_timestep = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time)) {
      max_comp_time = std::stoi(results[1].str());
      continue;
    }
    // read initial/goal nodes
    if (std::regex_match(line, results, r_sg) &&
        scen_type == ScenarioType::USER_SPECIFIED &&
        config_s.size() < num_agents) {
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      int x_g = std::stoi(results[3].str());
      int y_g = std::stoi(results[4].str());
      if (!G->existNode(x_s, y_s)) {
        halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
             ") does not exist, invalid scenario");
      }
      if (!G->existNode(x_g, y_g)) {
        halt("goal node (" + std::to_string(x_g) + ", " + std::to_string(y_g) +
             ") does not exist, invalid scenario");
      }

      Node* s = G->getNode(x_s, y_s);
      Node* g = G->getNode(x_g, y_g);
      config_s.push_back(s);
      config_g.push_back(g);
    }
  }

  // set default value not identified params
  if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
  if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;

  // check starts/goals
  if (num_agents <= 0) halt("invalid number of agents");
  if (!config_s.empty() && num_agents > config_s.size()) {
    warn("given starts/goals are not sufficient\nrandomly create instances");
  }
  if (num_agents > config_s.size() &&
      scen_type == ScenarioType::USER_SPECIFIED) {
    scen_type = ScenarioType::RANDOM;
  }

  if (scen_type == ScenarioType::RANDOM) {
    setRandomStartsGoals(flocking_blocks);
  }

  // trimming
  config_s.resize(num_agents);
  config_g.resize(num_agents);
}

Problem::Problem(Problem* P, Config _config_s, Config _config_g,
                 int _max_comp_time, int _max_timestep)
    : G(P->getG()),
      MT(P->getMT()),
      config_s(_config_s),
      config_g(_config_g),
      num_agents(P->getNum()),
      max_timestep(_max_timestep),
      max_comp_time(_max_comp_time),
      instance_initialized(false)
{
}

Problem::~Problem()
{
  config_s.clear();
  config_g.clear();

  if (instance_initialized) {
    delete G;
    delete MT;
  }
}

Node* Problem::getStart(int i) const
{
  if (!(0 <= i && i < config_s.size())) halt("invalid index");
  return config_s[i];
}

Node* Problem::getGoal(int i) const
{
  if (!(0 <= i && i < config_g.size())) halt("invalid index");
  return config_g[i];
}

void Problem::setRandomStartsGoals(const int flocking_blocks)
{
  const int group_num = (flocking_blocks <= 0 || flocking_blocks > num_agents)
                            ? num_agents
                            : flocking_blocks;

  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  Grid* grid = reinterpret_cast<Grid*>(G);
  const int N = grid->getWidth() * grid->getHeight();

  // set seeds of starts
  std::vector<Node*> seed_starts, seed_goals;
  for (int i = 0; i < group_num; ++i) {
    Node* s = nullptr;
    while (s == nullptr || inArray(s, seed_starts)) {
      s = G->getNode(getRandomInt(0, N - 1, MT));
    }
    seed_starts.push_back(s);
    Node* g = nullptr;
    while (g == nullptr || inArray(g, seed_goals)) {
      g = G->getNode(getRandomInt(0, N - 1, MT));
    }
    seed_goals.push_back(g);
  }

  int i = 0;
  while (config_s.size() < num_agents) {
    while (inArray(seed_starts[i], config_s)) {
      seed_starts[i] = randomChoose(seed_starts[i]->neighbor, MT);
    }
    config_s.push_back(seed_starts[i]);
    while (inArray(seed_goals[i], config_g)) {
      seed_goals[i] = randomChoose(seed_goals[i]->neighbor, MT);
    }
    config_g.push_back(seed_goals[i]);
    i = (i + 1) % group_num;
  }
}

void Problem::makeScenFile(const std::string& output_file)
{
  Grid* grid = reinterpret_cast<Grid*>(G);
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "agents=" << num_agents << "\n";
  log << "seed=0\n";
  log << "random_problem=0\n";
  log << "max_timestep=" << max_timestep << "\n";
  log << "max_comp_time=" << max_comp_time << "\n";
  for (int i = 0; i < num_agents; ++i) {
    log << config_s[i]->pos.x << "," << config_s[i]->pos.y << ","
        << config_g[i]->pos.x << "," << config_g[i]->pos.y << "\n";
  }
  log.close();
}
