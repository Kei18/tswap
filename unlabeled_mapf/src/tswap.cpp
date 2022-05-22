#include "../include/tswap.hpp"

#include <alloca.h>

#include <fstream>

const std::string TSWAP::SOLVER_NAME = "TSWAP";

TSWAP::TSWAP(Problem* _P)
    : Solver(_P),
      assignment_mode(GoalAllocator::BOTTLENECK_LINEAR),
      goal_indexes(G->getNodesSize(), -1)
{
  solver_name = SOLVER_NAME;
  for (int i = 0; i < P->getNum(); ++i) goal_indexes[P->getGoal(i)->id] = i;
}

TSWAP::~TSWAP() {}

/*
 * The following implementation is equivalent to naive-tswap
 * but changes the order of planning agents according to the situation.
 * This modification improves the solution quality of TSWAP.
 */
void TSWAP::run()
{
  Plan plan;  // will be solution

  // goal assignment
  info(" ", "start task allocation");
  allocator = std::make_shared<GoalAllocator>(P, assignment_mode);
  allocator->assign();
  auto goals = allocator->getAssignedGoals();

  elapsed_assignment = getSolverElapsedTime();
  estimated_soc = allocator->getCost();
  estimated_makespan = allocator->getMakespan();

  info(" ", "elapsed:", elapsed_assignment, ", finish goal assignment",
       ", soc: >=", estimated_soc, ", makespan: >=", estimated_makespan);

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
  std::vector<Agent*> occupied_now(G->getNodesSize(),
                                   nullptr);  // current location
  std::vector<Agent*> occupied_next(G->getNodesSize(),
                                    nullptr);  // next location

  // actions
  auto moveTo = [&](Agent* a, Node* v) {
    a->v_next = v;
    occupied_next[v->id] = a;
  };
  auto stay = [&](Agent* a) { moveTo(a, a->v_now); };
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
    a->called = 0;  // how many times an agent is called in the queue
    occupied_now[a->v_now->id] = a;

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

      // get desired node
      Node* u = getNextNode(a_i->v_now, a_i->g);

      // rule 2. if u is occupied in the *next* timestep -> stay
      auto a_j = occupied_next[u->id];
      if (a_j != nullptr) {
        if (a_j->v_next == a_j->g) swapGoal(a_i, a_j);
        stay(a_i);
        continue;
      }

      // rule 3. if u is occupied in the *current* timestep
      a_j = occupied_now[u->id];
      if (a_j != nullptr) {
        // To understand the following code, I recommend you to write
        // illustrations. The key point is that a_i sometimes can move even
        // after swapping/rotating targets.
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

    // acting
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr);
    for (int i = 0; i < P->getNum(); ++i) {
      Agent* a = &(A[i]);
      // clear
      occupied_next[a->v_next->id] = nullptr;
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      // set next location
      config[i] = a->v_next;
      occupied_now[a->v_next->id] = a;
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

  info(" ", "elapsed:", getSolverElapsedTime(), ", finish path planning");

  solution = plan;
}

Node* TSWAP::getNextNode(Node* a, Node* b)
{
  int i = goal_indexes[b->id];
  int cost_baseline = allocator->getLazyEval(a, i);
  for (auto m : a->neighbor) {
    if (m == b) return b;  // goal
    if (allocator->getLazyEval(m, i) < cost_baseline) return m;
  }
  return a;
}

bool TSWAP::deadlockDetectResolve(Agent* a, std::vector<Agent*>& occupied_now)
{
  // deadlock detection
  std::vector<Agent*> A_p;
  Agent* b = a;
  while (true) {
    if (b->v_now == b->g || b->v_next != nullptr) break;  // not deadlock
    auto c = occupied_now[getNextNode(b->v_now, b->g)->id];
    if (c == nullptr) break;  // not deadlock
    A_p.push_back(b);
    b = c;
    if (A_p.size() > 1) {
      if (b == a) break;  // deadlock

      // there is a deadlock, but "a" is not in the deadlock
      if (inArray(b, A_p)) {
        A_p.clear();
        break;
      }
    }
  }
  if (A_p.size() > 1 && b == a) {  // when detecting deadlock
    // rotate targets
    Node* g = (*(A_p.end() - 1))->g;
    for (auto itr = A_p.end() - 1; itr != A_p.begin(); --itr)
      (*itr)->g = (*(itr - 1))->g;
    (*A_p.begin())->g = g;
    return true;
  }

  return false;
}

void TSWAP::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"mode", no_argument, 0, 'm'},
      {"off-tie-break", no_argument, 0, 'b'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "m:", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'm':
        assignment_mode = static_cast<GoalAllocator::MODE>(std::atoi(optarg));
        break;
      default:
        break;
    }
  }
}

void TSWAP::printHelp()
{
  std::cout
      << TSWAP::SOLVER_NAME << "\n"

      << "  -m --mode"
      << "                     "
      << "assignment mode\n"
      << "                                    0: bottleneck-linear (default)\n"
      << "                                    1: bottleneck-linear (without "
         "lazy eval)\n"
      << "                                    2: bottleneck\n"
      << "                                    3: linear\n"
      << "                                    4: greedy\n"
      << "                                    5: greedy-swap\n"
      << "                                    6: greedy-swap (without lazy "
         "eval)\n"
      << "                                    7: greedy-swap-cost"

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
