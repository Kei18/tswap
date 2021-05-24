#include "../include/lib_ten.hpp"

#include <stack>

LibTEN::TEN_Node::TEN_Node(NodeType _type, Node* _v, int _t)
    : type(_type), v(_v), t(_t)
{
  name = getName(type, _v, t);
}

void LibTEN::TEN_Node::addParent(LibTEN::TEN_Node* parent)
{
  parents.push_back(parent);
  parent->children.push_back(this);
}

void LibTEN::TEN_Node::removeParent(LibTEN::TEN_Node* parent)
{
  auto itr1 = std::find(parents.begin(), parents.end(), parent);
  if (itr1 == parents.end()) return;
  parents.erase(itr1);
  auto itr2 = std::find(parent->children.begin(), parent->children.end(), this);
  parent->children.erase(itr2);
}

std::string LibTEN::TEN_Node::getName(NodeType _type, Node* _v, int _t)
{
  switch (_type) {
    case NodeType::SOURCE:
      return "source";
      break;
    case NodeType::SINK:
      return "sink";
      break;
    case NodeType::V_IN:
    case NodeType::V_OUT:
      return std::to_string(_v->id) + "_" + std::to_string(_t) + "_" +
             std::to_string(_type);
      break;
    default:
      halt("invalid operation, unknown LibTEN::TEN_Node type");
      return "";
  }
}

LibTEN::ResidualNetwork::ResidualNetwork()
    : apply_filter(false),
      use_ilp_solver(false),
      time_limit(-1),
      dfs_cnt(0),
      variants_cnt(0),
      constraints_cnt(0)
{
  init();
}

LibTEN::ResidualNetwork::ResidualNetwork(bool _filter, bool _ilp, Problem* _P)
    : apply_filter(_filter),
      use_ilp_solver(_ilp),
      P(_P),
      time_limit(-1),
      dfs_cnt(0),
      variants_cnt(0),
      constraints_cnt(0)
{
  init();
  if (apply_filter && !use_ilp_solver) createFilter();
}

LibTEN::ResidualNetwork::~ResidualNetwork()
{
  for (auto itr = body.begin(); itr != body.end(); ++itr) delete itr->second;
  body.clear();
  capacity.clear();
}

void LibTEN::ResidualNetwork::init()
{
  source = createNewNode(LibTEN::TEN_Node::SOURCE);
  sink = createNewNode(LibTEN::TEN_Node::SINK);

#ifdef _GUROBI_
  if (use_ilp_solver) {
    grb_env = std::make_unique<GRBEnv>(true);
    grb_env->set("OutputFlag", "0");
    grb_env->start();
    grb_model = std::make_unique<GRBModel>(*grb_env);
  }
#endif
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode(
    TEN_Node::NodeType _type, Node* _v, int _t)
{
  LibTEN::TEN_Node* new_node = new LibTEN::TEN_Node(_type, _v, _t);
  auto itr = body.find(new_node->name);
  if (itr != body.end()) delete itr->second;
  body[new_node->name] = new_node;
  return new_node;
}

// used for sink or source
LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode(
    TEN_Node::NodeType _type)
{
  if ((_type != TEN_Node::NodeType::SOURCE) &&
      (_type != TEN_Node::NodeType::SINK)) {
    halt("invalid type");
  }
  return createNewNode(_type, nullptr, 0);
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::getNode(NodeType _type, Node* _v,
                                                   Node* _u, int _t)
{
  auto itr = body.find(TEN_Node::getName(_type, _v, _t));
  return (itr != body.end()) ? itr->second : nullptr;
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::getNode(NodeType _type, Node* _v,
                                                   int _t)
{
  return getNode(_type, _v, nullptr, _t);
}

std::string LibTEN::ResidualNetwork::getEdgeName(LibTEN::TEN_Node* p,
                                                 LibTEN::TEN_Node* q)
{
  return p->name + "__" + q->name;
}

// TEN_Node 1.name__TEN_Node 2.name -> TEN_Node 2.name -- TEN_Node 1.name
std::string LibTEN::ResidualNetwork::getReverseEdgeName(const std::string s)
{
  for (int i = 0; i < s.size() - 1; ++i) {
    if (s[i] == '_' && s[i + 1] == '_') {
      return s.substr(i + 2, s.size()) + "__" + s.substr(0, i);
    }
  }
  return "";
}

int LibTEN::ResidualNetwork::getNodesNum() { return body.size(); }

int LibTEN::ResidualNetwork::getEdgesNum()
{
  return std::accumulate(body.begin(), body.end(), 0,
                         [](int acc, decltype(body)::value_type& itr) {
                           return acc + itr.second->children.size();
                         });
}

int LibTEN::ResidualNetwork::getCapacity(TEN_Node* p, TEN_Node* q)
{
  std::string key = getEdgeName(p, q);
  auto itr = capacity.find(key);
  if (itr != capacity.end()) return itr->second;

  // require initialization
  initEdge(p, q);
  return capacity[key];
}

void LibTEN::ResidualNetwork::clearAllCapacity() { capacity.clear(); }

void LibTEN::ResidualNetwork::initEdge(TEN_Node* p, TEN_Node* q)
{
  std::string key1 = getEdgeName(p, q);
  std::string key2 = getEdgeName(q, p);

  // capacity \in { 0, 1 }
  if (inArray(q, p->children)) {
    capacity[key1] = 1;
    capacity[key2] = 0;
  } else if (inArray(q, p->parents)) {
    capacity[key1] = 0;
    capacity[key2] = 1;
  } else {
    halt("invalid residual capacity: " + p->name + " -> " + q->name);
  }
}

void LibTEN::ResidualNetwork::deleteEdge(TEN_Node* p, TEN_Node* q)
{
  capacity.erase(getEdgeName(p, q));
  capacity.erase(getEdgeName(q, p));
}

void LibTEN::ResidualNetwork::increment(TEN_Node* p, TEN_Node* q)
{
  capacity[getEdgeName(p, q)] = 1;
}

void LibTEN::ResidualNetwork::decrement(TEN_Node* p, TEN_Node* q)
{
  capacity[getEdgeName(p, q)] = 0;
}

void LibTEN::ResidualNetwork::setFlow(TEN_Node* from, TEN_Node* to)
{
  decrement(from, to);
  increment(to, from);
}

void LibTEN::ResidualNetwork::setFlow(const std::string edge_name)
{
  capacity[edge_name] = 0;
  capacity[getReverseEdgeName(edge_name)] = 1;
}

void LibTEN::ResidualNetwork::setReverseFlow(const std::string edge_name)
{
  capacity[edge_name] = 1;
  capacity[getReverseEdgeName(edge_name)] = 0;
}

int LibTEN::ResidualNetwork::getFlowSum()
{
  return std::accumulate(
      source->children.begin(), source->children.end(), 0,
      [&](int acc, TEN_Node* p) { return acc + getCapacity(p, source); });
}

void LibTEN::ResidualNetwork::solve()
{
#ifdef _GUROBI_
  if (use_ilp_solver) {
    solveByGUROBI();
  } else {
    FordFulkerson();
  }
#else
  FordFulkerson();
#endif
}

/*
 * Notice about implementation of DFS.
 * Recursive call is much faster, however,
 * when problems become huge enough, I found stack overflows.
 * Therefore, the following implementation switches the modes
 * according to the number of vertices
 */

void LibTEN::ResidualNetwork::FordFulkerson()
{
  constexpr unsigned int MEMORY_LIMIT = 2000000;  // arbitrary value
  if (getNodesNum() > MEMORY_LIMIT) {
    FordFulkersonWithStack();
  } else {
    FordFulkersonWithRecursiveCall();
  }
}

void LibTEN::ResidualNetwork::FordFulkersonWithStack()
{
  dfs_cnt = 0;

  struct DFSNode {
    TEN_Node* v;
    DFSNode* p;  // parent, for backtracking
  };

  while (true) {
    // depth first search
    std::stack<DFSNode*> OPEN;
    std::unordered_map<TEN_Node*, bool> CLOSE;

    std::vector<DFSNode*> GC;  // garbage collection
    auto createNewNode = [&](TEN_Node* v, DFSNode* p) {
      auto q = new DFSNode{v, p};
      GC.push_back(q);
      return q;
    };

    // setup initial node
    OPEN.push(createNewNode(source, nullptr));

    // goal dfs node
    DFSNode* bottom = nullptr;

    // main loop
    while (!OPEN.empty()) {
      ++dfs_cnt;

      auto top = OPEN.top();
      auto p = top->v;
      OPEN.pop();

      // check close list
      if (CLOSE.find(p) != CLOSE.end()) continue;

      // update CLOSE
      CLOSE[p] = true;

      // reach goal
      if (p == sink) {
        bottom = top;
        break;
      }

      // set neighbors
      LibTEN::TEN_Nodes next;
      for (auto itr = std::rbegin(p->parents); itr != std::rend(p->parents);
           ++itr)
        next.push_back(*itr);
      for (auto itr = std::rbegin(p->children); itr != std::rend(p->children);
           ++itr)
        next.push_back(*itr);

      // expand
      for (auto q : next) {
        // already searched
        if (CLOSE.find(q) != CLOSE.end()) continue;

        // used
        if (getCapacity(p, q) == 0) continue;

        // pruning
        if (apply_filter) {
          if (p->type == NodeType::SOURCE && q->type == NodeType::V_IN) {
            if (reachable_filter[q->v] > sink->t) continue;
          } else if (p->type == NodeType::V_IN && q->type == NodeType::V_OUT) {
            if (reachable_filter[q->v] + q->t > sink->t) continue;
          }
        }

        OPEN.push(createNewNode(q, top));
      }
    }

    // success
    if (bottom != nullptr) {
      // backtracking
      auto q = bottom;
      while (q->p != nullptr) {
        setFlow(q->p->v, q->v);
        q = q->p;
      }
    }

    // free
    for (auto p : GC) delete p;

    // end
    if (bottom == nullptr) break;
  }
}

/*
 * This is recursive version
 */
void LibTEN::ResidualNetwork::FordFulkersonWithRecursiveCall()
{
  dfs_cnt = 0;

  while (true) {
    // depth first search
    std::unordered_map<TEN_Node*, bool> CLOSED;
    auto dfs = [&](auto&& self, TEN_Node* p) -> TEN_Node* {
      // check closed list
      if (CLOSED[p]) return nullptr;

      // update closed list
      CLOSED[p] = true;
      ++dfs_cnt;

      // reach goal
      if (p == sink) return p;

      // check children
      LibTEN::TEN_Nodes next;
      next.insert(next.end(), p->children.begin(), p->children.end());
      next.insert(next.end(), p->parents.begin(), p->parents.end());
      for (auto q : next) {
        // already searched
        if (CLOSED.find(q) != CLOSED.end()) continue;

        // used
        if (getCapacity(p, q) == 0) continue;

        // pruning
        if (apply_filter) {
          if (p->type == NodeType::SOURCE && q->type == NodeType::V_IN) {
            if (reachable_filter[q->v] > sink->t) continue;
          } else if (p->type == NodeType::V_IN && q->type == NodeType::V_OUT) {
            if (reachable_filter[q->v] + q->t > sink->t) continue;
          }
        }

        // recursive call
        auto res = self(self, q);

        // success
        if (res != nullptr) {
          setFlow(p, q);
          return res;
        }
      }

      // failed
      return nullptr;
    };

    if (dfs(dfs, source) == nullptr) break;
  }
}

// for pruning
void LibTEN::ResidualNetwork::createFilter()
{
  std::vector<Node*> OPEN, OPEN_NEXT;

  // initialize
  int t = 0;  // timestep
  for (auto v : P->getConfigGoal()) {
    OPEN.push_back(v);
    reachable_filter[v] = t;
  }
  // bfs
  while (true) {
    ++t;
    for (auto v : OPEN) {
      for (auto u : v->neighbor) {
        if (reachable_filter.find(u) != reachable_filter.end()) continue;
        OPEN_NEXT.push_back(u);
        reachable_filter[u] = t;
      }
    }

    if (OPEN_NEXT.empty()) break;

    OPEN = OPEN_NEXT;
    OPEN_NEXT.clear();
  }
}

void LibTEN::ResidualNetwork::addParent(TEN_Node* child, TEN_Node* parent)
{
  child->addParent(parent);

#ifdef _GUROBI_
  if (use_ilp_solver) {
    // add variable
    auto name = getEdgeName(parent, child);
    auto var = grb_model->addVar(0.0, 1.0, 0, GRB_BINARY, name);
    grb_table_vars[name] = var;

    // set objective
    if (parent == source && source->children.size() == P->getNum()) {
      GRBLinExpr grb_obj = 0;
      for (auto p : source->children) {
        grb_obj += grb_table_vars[getEdgeName(source, p)];
      }
      grb_model->setObjective(grb_obj, GRB_MAXIMIZE);
    }

    // add constraints
    auto updateConstr = [&](TEN_Node* p) {
      if (p == source || p == sink) return;

      // check update condition
      if ((p->type == NodeType::V_OUT &&
           p->parents.size() == p->v->neighbor.size() + 1) ||
          (p->type == NodeType::V_IN &&
           p->children.size() == p->v->neighbor.size() + 1)) {
        // update
        auto itr = grb_table_constr.find(p->name);
        if (itr != grb_table_constr.end()) grb_model->remove(itr->second);
        GRBLinExpr lhs = 0;
        for (auto q : p->children) lhs += grb_table_vars[getEdgeName(p, q)];
        for (auto q : p->parents) lhs -= grb_table_vars[getEdgeName(q, p)];
        grb_table_constr[p->name] = grb_model->addConstr(lhs == 0, p->name);
      }
    };

    updateConstr(parent);
    updateConstr(child);

    variants_cnt = grb_table_vars.size();
    constraints_cnt = grb_table_constr.size();
  }
#endif
}

void LibTEN::ResidualNetwork::removeParent(TEN_Node* child, TEN_Node* parent)
{
  child->removeParent(parent);

#ifdef _GUROBI_
  if (use_ilp_solver) {
    // remove variable
    auto itr = grb_table_vars.find(getEdgeName(parent, child));
    if (itr != grb_table_vars.end()) {
      grb_model->remove(itr->second);
      grb_table_vars.erase(itr);
    }

    variants_cnt = grb_table_vars.size();
    constraints_cnt = grb_table_constr.size();
  }
#endif
}

#ifdef _GUROBI_
void LibTEN::ResidualNetwork::solveByGUROBI()
{
  // set time limit
  if (time_limit != -1) {
    grb_model->set("TimeLimit", std::to_string((double)time_limit / 1000));
  }

  // optimize model
  grb_model->optimize();

  // feasible solution
  if (grb_model->get(GRB_DoubleAttr_ObjVal) == P->getNum()) {
    // apply flow
    for (auto itr = grb_table_vars.begin(); itr != grb_table_vars.end();
         ++itr) {
      if (itr->second.get(GRB_DoubleAttr_X) == 1.0) {
        setFlow(itr->first);
      } else {
        setReverseFlow(itr->first);
      }
    }
  }
}
#endif
