#include "../include/lib_ten.hpp"

#include <queue>

LibTEN::TEN_Node::TEN_Node(NodeType _type, Node* _v, Node* _u, int _t)
    : type(_type), v(_v), u(_u), t(_t)
{
  name = getName(type, _v, _u, t);
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

std::string LibTEN::TEN_Node::getName
(NodeType _type, Node* _v, Node* _u, int _t)
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
    case NodeType::W_IN:
    case NodeType::W_OUT:
      if (_u == nullptr) halt("invalid operation, LibTEN::TEN_Node::getName");
      return std::to_string(_v->id) + "_" + std::to_string(_u->id) + "_" +
             std::to_string(_t) + "_" + std::to_string(_type);
    default:
      halt("invalid operation, unknown LibTEN::TEN_Node type");
      return "";
  }
}

std::string LibTEN::TEN_Node::getName(NodeType _type, Node* _v, int _t)
{
  return getName(_type, _v, nullptr, _t);
}

std::string LibTEN::TEN_Node::getStr()
{
  if (type == NodeType::SOURCE) return "source";
  if (type == NodeType::SINK) return "sink";

  std::string str;
  if (type == NodeType::V_IN)  str += "V_IN ";
  if (type == NodeType::V_OUT) str += "V_OUT";
  if (type == NodeType::W_IN)  str += "W_IN ";
  if (type == NodeType::W_OUT) str += "W_OUT";
  str += ", t: " + std::to_string(t);
  str += ", v: " + std::to_string(v->id);
  if (u != nullptr) str += ", u: " + std::to_string(u->id);
  return str;
}

LibTEN::ResidualNetwork::ResidualNetwork()
  : apply_filter(false),
    dfs_cnt(0)
{
  init();
}

LibTEN::ResidualNetwork::ResidualNetwork(bool _filter, Problem* _P)
  : apply_filter(_filter),
    P(_P),
    dfs_cnt(0)
{
  init();
  if (apply_filter) createFilter();
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
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode(
    TEN_Node::NodeType _type, Node* _v, Node* _u, int _t)
{
  LibTEN::TEN_Node* new_node = new LibTEN::TEN_Node(_type, _v, _u, _t);
  body[new_node->name] = new_node;
  return new_node;
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode(
    TEN_Node::NodeType _type, Node* _v, int _t)
{
  return createNewNode(_type, _v, nullptr, _t);
}

// used for sink or source
LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode(
    TEN_Node::NodeType _type)
{
  if ((_type != TEN_Node::NodeType::SOURCE) &&
      (_type != TEN_Node::NodeType::SINK)) {
    halt("invalid type");
  }
  return createNewNode(_type, nullptr, nullptr, 0);
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::getNode(NodeType _type, Node* _v,
                                                   Node* _u, int _t)
{
  auto itr = body.find(TEN_Node::getName(_type, _v, _u, _t));
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
  int cap = getCapacity(p, q);
  capacity[getEdgeName(p, q)] = cap + 1;
}

void LibTEN::ResidualNetwork::decrement(TEN_Node* p, TEN_Node* q)
{
  int cap = getCapacity(p, q);
  capacity[getEdgeName(p, q)] = cap - 1;
}

void LibTEN::ResidualNetwork::setFlow(TEN_Node* from, TEN_Node* to)
{
  decrement(from, to);
  increment(to, from);
}

int LibTEN::ResidualNetwork::getFlowSum()
{
  return std::accumulate(
      source->children.begin(), source->children.end(), 0,
      [&](int acc, TEN_Node* p) { return acc + getCapacity(p, source); });
}

void LibTEN::ResidualNetwork::FordFulkerson()
{
  dfs_cnt = 0;

  while (true) {
    // depth first search
    std::unordered_map<TEN_Node*, bool> CLOSED;
    auto dfs = [&](auto&& self, TEN_Node* p) -> TEN_Node* {
      ++dfs_cnt;

      // update closed list
      CLOSED[p] = true;

      // reach goal
      if (p == sink) return p;

      // check children
      LibTEN::TEN_Nodes next;
      next.insert(next.end(), p->children.begin(), p->children.end());
      next.insert(next.end(), p->parents.begin(), p->parents.end());
      for (auto q : next) {
        // already searched
        if (CLOSED.find(q) != CLOSED.end()) continue;

        // fulfill
        if (getCapacity(p, q) == 0) continue;

        // apply filter
        if (apply_filter) {
          if (p->type == NodeType::SOURCE && q->type == NodeType::V_IN) {
            if (reachable_filter[q->v] > sink->t) continue;
          } else if ((p->type == NodeType::V_IN || p->type == NodeType::W_OUT)
              && q->type == NodeType::V_OUT) {
            if (reachable_filter[q->v] + q->t > sink->t) continue;
          } else if (p->type == NodeType::V_IN && q->type == NodeType::W_IN) {
            auto u = (q->v == p->v) ? q->u : q->v;
            if (reachable_filter[u] + q->t > sink->t) continue;
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

void LibTEN::ResidualNetwork::createFilter()
{
  // bfs
  std::vector<Node*> OPEN, OPEN_NEXT;

  // initialize
  int t = 0;
  for (auto v : P->getConfigGoal()) {
    OPEN.push_back(v);
    reachable_filter[v] = t;
  }
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
