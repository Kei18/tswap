#include "../include/tswap.hpp"

#include "../include/goal_allocator.hpp"
#include <fstream>

const std::string TSWAP::SOLVER_NAME = "TSWAP";

TSWAP::TSWAP(Problem* _P)
  : Solver(_P),
    use_bfs_allocate(false)
{
  solver_name = SOLVER_NAME;
}

TSWAP::~TSWAP() {}

void TSWAP::run()
{
  Plan plan;  // will be solution

  // goal assignment
  info(" ", "start task allocation");
  GoalAllocator allocator = GoalAllocator(P, use_bfs_allocate);
  allocator.assign();
  auto goals = allocator.getAssignedGoals();

  elapsed_assignment = getSolverElapsedTime();
  estimated_soc = allocator.getCost();
  estimated_makespan = allocator.getMakespan();

  info(" ", "elapsed:", elapsed_assignment,
       ", finish goal assignment",
       ", soc: >=", estimated_soc,
       ", makespan: >=", estimated_makespan);

  auto t_pathplanning = Time::now();

  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->called != b->called) return a->called > b->called;
    if (a->v_now != a->g) return false;
    if (b->v_now != b->g) return true;
    return a < b;
  };

  // agents have not decided their next locations
  std::priority_queue<Agent*, std::vector<Agent*>, decltype(compare)> undecided(
      compare);

  // work as reservation table
  std::unordered_map<Node*, Agent*> occupied_now;   // current location
  std::unordered_map<Node*, Agent*> occupied_next;  // next location

  // actions
  auto moveTo = [&](Agent* a, Node* v) {
    a->v_next = v;
    occupied_next[v] = a;
  };
  auto stay = [&](Agent* a) { moveTo(a, a->v_now); };

  // change goal
  auto swapGoal = [&](Agent* a, Agent* b) {
    Node* v = b->g;
    b->g = a->g;
    a->g = v;
  };

  // all agents
  Agent A[P->getNum()];

  // setup agents
  for (int i = 0; i < P->getNum(); ++i) {
    auto a = &(A[i]);
    a->id = i;                  // id
    a->v_now = P->getStart(i);  // current node
    a->v_next = nullptr;        // next node
    a->g = goals[i];            // goal
    a->called = 0;              // how many times an agent is called in the queue
    occupied_now[a->v_now] = a;

    // insert OPEN set
    undecided.push(a);
  }

  // set initial config
  plan.add(P->getConfigStart());

  // main loop
  int timestep = 0;
  while (true) {
    // planning
    while (!undecided.empty()) {
      // pickup one agent
      Agent* a_i = undecided.top();
      undecided.pop();
      a_i->called++;

      // rule 1. stay goal
      if (a_i->v_now == a_i->g) {
        stay(a_i);
        continue;
      }

      // desired node
      Node* u = planOneStep(a_i, occupied_now, occupied_next);

      // rule 2. if u is occupied in next timestep -> stay
      auto itr_next = occupied_next.find(u);
      if (itr_next != occupied_next.end()) {
        Agent* a_j = itr_next->second;
        if (a_j->v_next == a_j->g) swapGoal(a_i, a_j);
        stay(a_i);
        continue;
      }

      // rule 3. if u is occupied in the current timestep
      auto itr_now = occupied_now.find(u);
      if (itr_now != occupied_now.end()) {
        Agent* a_j = itr_now->second;
        if (a_j->v_now == a_j->g) {
          swapGoal(a_i, a_j);
        } else if (deadlockDetectResolve(a_i, occupied_now)) {  // deadlock detection
          // skip
          undecided.push(a_i);
          continue;
        }

        if (a_j->v_next == nullptr) {
          // skip
          undecided.push(a_i);
        } else if (a_j->v_next == u) {
          // wait
          stay(a_i);
        } else {
          // move
          moveTo(a_i, u);
        }
        continue;
      }

      // rule4. otherwise (u is free)
      moveTo(a_i, u);
      continue;
    }

    // clear
    occupied_now.clear();
    occupied_next.clear();

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (int i = 0; i < P->getNum(); ++i) {
      Agent* a = &(A[i]);
      // set next location
      config[i] = a->v_next;
      occupied_now[a->v_next] = a;
      // check goal condition
      check_goal_cond &= (a->v_next == a->g);
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;
      a->called = 0;
      // push to priority queue
      undecided.push(a);
    }

    // update plan
    plan.add(config);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  elapsed_pathplanning = getElapsedTime(t_pathplanning);

  info(" ", "elapsed:", getSolverElapsedTime(), ", finish paht planning");

  solution = plan;
}

bool TSWAP::deadlockDetectResolve
(Agent* a, std::unordered_map<Node*, Agent*>& occupied_now)
{
  // deadlock detection
  std::vector<Agent*> A_p;
  Agent* b = a;
  while (true) {
    if (b->v_now == b->g || b->v_next != nullptr) break;  // not deadlock
    auto itr = occupied_now.find(getPath(b->v_now, b->g)[1]);
    if (itr == occupied_now.end()) break;  // not deadlock
    A_p.push_back(b);
    b = itr->second;
    if (A_p.size() > 1 && b == a) break;  // deadlock
  }
  if (A_p.size() > 1 && b == a) {  // detect deadlock
    // rotate targets
    Node* g = (*(A_p.end()-1))->g;
    for (auto itr = A_p.begin()+1; itr != A_p.end(); ++itr) (*itr)->g = (*(itr-1))->g;
    (*A_p.begin())->g = g;
    return true;
  }

  return false;
}

Node* TSWAP::planOneStep(Agent* a,
                         std::unordered_map<Node*, Agent*>& occupied_now,
                         std::unordered_map<Node*, Agent*>& occupied_next)
{
  Nodes C = a->v_now->neighbor;

  // goal exists -> return immediately
  if (inArray(a->g, C)) return a->g;

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  return *std::min_element(C.begin(), C.end(), [&](Node* v, Node* u) {
    // path distance
    int c_v = pathDist(v, a->g);
    int c_u = pathDist(u, a->g);
    if (c_v != c_u) return c_v < c_u;
    // tiebreak1. occupancy for next timestep
    int o_v_next = (int)(occupied_next.find(v) != occupied_next.end());
    int o_u_next = (int)(occupied_next.find(u) != occupied_next.end());
    if (o_v_next != o_u_next) return o_v_next < o_u_next;
    // tiebreak2. occupancy for current timestep
    int o_v_now = (int)(occupied_now.find(v) != occupied_now.end());
    int o_u_now = (int)(occupied_now.find(u) != occupied_now.end());
    if (o_v_now != o_u_now) return o_v_now < o_u_now;
    return getRandomBoolean(MT);
  });
}

void TSWAP::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
    {"use-bfs-allocate", no_argument, 0, 'b'},
    {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "b", longopts, &longindex)) != -1) {
    switch (opt) {
    case 'b':
      use_bfs_allocate = true;
      break;
    default:
      break;
    }
  }
}

void TSWAP::printHelp()
{
  std::cout << TSWAP::SOLVER_NAME << "\n"

            << "  -b --use-bfs-allocate"
            << "         "
            << "use BFS in goal allocation"

            << std::endl;
}

void TSWAP::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  log << "internal_info=\n"
      << "elapsed_assignment:" << elapsed_assignment << "\n"
      << "elapsed_path_planning:" << elapsed_pathplanning << "\n"
      << "estimated_soc:" << estimated_soc << "\n"
      << "estimated_makespan:" << estimated_makespan << "\n";

  makeLogSolution(log);
  log.close();
}
