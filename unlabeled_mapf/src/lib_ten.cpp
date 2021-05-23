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
      time_limit(-1),
      dfs_cnt(0)
{
  init();
}

LibTEN::ResidualNetwork::ResidualNetwork(bool _filter, Problem* _P)
    : apply_filter(_filter),
      P(_P),
      time_limit(-1),
      dfs_cnt(0)
{
  init();
  if (apply_filter) createFilter();
}

LibTEN::ResidualNetwork::~ResidualNetwork()
{
  for (int t = 0; t < (int)body_V_IN.size(); ++t) {
    for (auto node : body_V_IN[t])  delete node;
    for (auto node : body_V_OUT[t]) delete node;
  }
  delete source;
  delete sink;
  capacity.clear();
}

void LibTEN::ResidualNetwork::init()
{
  source = createNewNode(LibTEN::TEN_Node::SOURCE);
  sink = createNewNode(LibTEN::TEN_Node::SINK);
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode(
    TEN_Node::NodeType _type, Node* _v, int _t)
{
  LibTEN::TEN_Node* new_node = new LibTEN::TEN_Node(_type, _v, _t);

  // extend
  if (_type == TEN_Node::NodeType::V_IN || _type == TEN_Node::NodeType::V_OUT) {
    if ((int)body_V_IN.size() < _t) {
      auto nodes_num = P->getG()->getNodesSize();
      body_V_IN.push_back(std::vector<TEN_Node*>(nodes_num, nullptr));
      body_V_OUT.push_back(std::vector<TEN_Node*>(nodes_num, nullptr));
    }
  }

  // register
  if (_type == TEN_Node::NodeType::V_IN) {
    body_V_IN[_t-1][_v->id] = new_node;
  } else if (_type == TEN_Node::NodeType::V_OUT) {
    body_V_OUT[_t-1][_v->id] = new_node;
  }

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
  switch (_type) {
  case TEN_Node::NodeType::SOURCE:
    return source;
  case TEN_Node::NodeType::SINK:
    return sink;
  case TEN_Node::NodeType::V_IN:
    return ((int)body_V_IN.size() >= _t) ? body_V_IN[_t-1][_v->id] : nullptr;
  case TEN_Node::NodeType::V_OUT:
    return ((int)body_V_OUT.size() >= _t) ? body_V_OUT[_t-1][_v->id] : nullptr;
  default:
    return nullptr;
  }
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::getNode(NodeType _type, Node* _v,
                                                   int _t)
{
  return getNode(_type, _v, nullptr, _t);
}

std::string LibTEN::ResidualNetwork::getEdgeName(LibTEN::TEN_Node* p,
                                                 LibTEN::TEN_Node* q)
{
  if (inArray(q, p->children)) {  // p-> q
    return p->name + "__" + q->name;
  } else if (inArray(q, p->parents)) {
    return q->name + "__" + p->name;
  }
  return "";
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

int LibTEN::ResidualNetwork::getNodesNum()
{
  int acc = 2;
  auto nodes_num = P->getG()->getNodesSize();
  for (int t = 0; t < (int)body_V_IN.size(); ++t) {
    for (int i = 0; i < nodes_num; ++i) {
      if (body_V_IN[t][i] != nullptr) ++acc;
      if (body_V_OUT[t][i] != nullptr) ++acc;
    }
  }
  return acc;
}

int LibTEN::ResidualNetwork::getEdgesNum()
{
  auto acc = source->children.size();
  auto nodes_num = P->getG()->getNodesSize();
  for (int t = 0; t < (int)body_V_IN.size(); ++t) {
    for (int i = 0; i < nodes_num; ++i) {
      acc += body_V_IN[t][i]->children.size();
      acc += body_V_OUT[t][i]->children.size();
    }
  }
  return acc;
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

bool LibTEN::ResidualNetwork::used(TEN_Node* p, TEN_Node* q)
{
  if (inArray(q, p->children)) {  // usual
    return !getCapacity(p, q);
  } else {  // reversed
    return getCapacity(p, q);
  }
}

void LibTEN::ResidualNetwork::clearAllCapacity() { capacity.clear(); }

void LibTEN::ResidualNetwork::initEdge(TEN_Node* p, TEN_Node* q)
{
  capacity[getEdgeName(p, q)] = true;
}

void LibTEN::ResidualNetwork::deleteEdge(TEN_Node* p, TEN_Node* q)
{
  capacity.erase(getEdgeName(p, q));
}

void LibTEN::ResidualNetwork::setFlow(TEN_Node* from, TEN_Node* to)
{
  auto key = getEdgeName(from, to);
  if (inArray(to, from->children)) {  // usual
    capacity[key] = false;
  } else {  // reverse
    capacity[key] = true;
  }
}

int LibTEN::ResidualNetwork::getFlowSum()
{
  return std::accumulate(
      source->children.begin(), source->children.end(), 0,
      [&](int acc, TEN_Node* p) { return acc + (1 - getCapacity(source, p)); });
}

void LibTEN::ResidualNetwork::solve()
{
  FordFulkerson();
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
        if (used(p, q)) continue;

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
        if (used(p, q)) continue;

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
}

void LibTEN::ResidualNetwork::removeParent(TEN_Node* child, TEN_Node* parent)
{
  child->removeParent(parent);
}
