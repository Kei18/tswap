#include "../include/lib_ten.hpp"


std::unordered_map<std::string, LibTEN::TEN_Node*> LibTEN::TEN_Node::all_nodes;

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

LibTEN::TEN_Node* LibTEN::TEN_Node::createNewNode(NodeType _type, Node* _v, Node* _u, int _t)
{
  LibTEN::TEN_Node* new_node = new LibTEN::TEN_Node(_type, _v, _u, _t);
  all_nodes[new_node->name] = new_node;
  return new_node;
}

LibTEN::TEN_Node* LibTEN::TEN_Node::createNewNode(NodeType _type, Node* _v, int _t)
{
  return createNewNode(_type, _v, nullptr, _t);
}

// used for sink or source
LibTEN::TEN_Node* LibTEN::TEN_Node::createNewNode(NodeType _type)
{
  if ((_type != NodeType::SOURCE) && (_type != NodeType::SINK)) halt("invalid type");
  return createNewNode(_type, nullptr, nullptr, 0);
}

std::string LibTEN::TEN_Node::getName(NodeType _type, Node* _v, Node* _u, int _t)
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
    return
      std::to_string(_v->id) + "_" +
      std::to_string(_t) + "_" +
      std::to_string(_type);
    break;
  case NodeType::W_IN:
  case NodeType::W_OUT:
    if (_u == nullptr) halt("invalid operation, LibTEN::TEN_Node::getName");
    return
      std::to_string(_v->id) + "_" +
      std::to_string(_u->id) + "_" +
      std::to_string(_t) + "_" +
      std::to_string(_type);
  default:
    halt("invalid operation, unknown LibTEN::TEN_Node type");
    return "";
  }
}

std::string LibTEN::TEN_Node::getName(NodeType _type, Node* _v, int _t)
{
  return getName(_type, _v, nullptr, _t);
}

std::string LibTEN::TEN_Node::getEdgeName(LibTEN::TEN_Node* p, LibTEN::TEN_Node* q)
{
  return p->name + "__" + q->name;
}

LibTEN::TEN_Node* LibTEN::TEN_Node::getNode(NodeType _type, Node* _v, Node* _u, int _t)
{
  auto itr = all_nodes.find(getName(_type, _v, _u, _t));
  return (itr != all_nodes.end()) ? itr->second : nullptr;
}

LibTEN::TEN_Node* LibTEN::TEN_Node::getNode(NodeType _type, Node* _v, int _t)
{
  return getNode(_type, _v, nullptr, _t);
}

int LibTEN::TEN_Node::getNodesNum()
{
  return all_nodes.size();
}

int LibTEN::TEN_Node::getEdgesNum()
{
  return std::accumulate(all_nodes.begin(), all_nodes.end(), 0,
                         [] (int acc, decltype(all_nodes)::value_type& itr)
                         { return acc + itr.second->children.size(); });
}

void LibTEN::TEN_Node::clear()
{
  for (auto itr = all_nodes.begin(); itr != all_nodes.end(); ++itr) delete itr->second;
  all_nodes.clear();
}

LibTEN::ResidualNetwork::ResidualNetwork()
{
  source = createNewNode(LibTEN::TEN_Node::SOURCE);
  sink   = createNewNode(LibTEN::TEN_Node::SINK);
}

LibTEN::ResidualNetwork::~ResidualNetwork()
{
  for (auto itr = body.begin(); itr != body.end(); ++itr) delete itr->second;
  body.clear();
  capacity.clear();
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode
(TEN_Node::NodeType _type, Node* _v, Node* _u, int _t)
{
  LibTEN::TEN_Node* new_node = new LibTEN::TEN_Node(_type, _v, _u, _t);
  body[new_node->name] = new_node;
  return new_node;
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode
(TEN_Node::NodeType _type, Node* _v, int _t)
{
  return createNewNode(_type, _v, nullptr, _t);
}

// used for sink or source
LibTEN::TEN_Node* LibTEN::ResidualNetwork::createNewNode
(TEN_Node::NodeType _type)
{
  if ((_type != TEN_Node::NodeType::SOURCE) &&
      (_type != TEN_Node::NodeType::SINK)) {
    halt("invalid type");
  }
  return createNewNode(_type, nullptr, nullptr, 0);
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::getNode
(TEN_Node::NodeType _type, Node* _v, Node* _u, int _t)
{
  auto itr = body.find(TEN_Node::getName(_type, _v, _u, _t));
  return (itr != body.end()) ? itr->second : nullptr;
}

LibTEN::TEN_Node* LibTEN::ResidualNetwork::getNode
(TEN_Node::NodeType _type, Node* _v, int _t)
{
  return getNode(_type, _v, nullptr, _t);
}

std::string LibTEN::ResidualNetwork::getEdgeName(LibTEN::TEN_Node* p, LibTEN::TEN_Node* q)
{
  return p->name + "__" + q->name;
}

int LibTEN::ResidualNetwork::getNodesNum()
{
  return body.size();
}

int LibTEN::ResidualNetwork::getEdgesNum()
{
  return std::accumulate(body.begin(), body.end(), 0,
                         [] (int acc, decltype(body)::value_type& itr)
                         { return acc + itr.second->children.size(); });
}

int LibTEN::ResidualNetwork::getCapacity(TEN_Node* p, TEN_Node* q)
{
  std::string key = LibTEN::TEN_Node::getEdgeName(p, q);
  auto itr = capacity.find(key);
  if (itr != capacity.end()) return itr->second;

  // require initialization
  initEdge(p, q);
  return capacity[key];
}

void LibTEN::ResidualNetwork::initEdge(TEN_Node* p, TEN_Node* q)
{
  std::string key1 = LibTEN::TEN_Node::getEdgeName(p, q);
  std::string key2 = LibTEN::TEN_Node::getEdgeName(q, p);

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
  capacity.erase(LibTEN::TEN_Node::getEdgeName(p, q));
  capacity.erase(LibTEN::TEN_Node::getEdgeName(q, p));
}

void LibTEN::ResidualNetwork::increment(TEN_Node* p, TEN_Node* q)
{
  int cap = getCapacity(p, q);
  capacity[LibTEN::TEN_Node::getEdgeName(p, q)] = cap + 1;
}

void LibTEN::ResidualNetwork::decrement(TEN_Node* p, TEN_Node* q)
{
  int cap = getCapacity(p, q);
  capacity[LibTEN::TEN_Node::getEdgeName(p, q)] = cap - 1;
}

void LibTEN::ResidualNetwork::setFlow(TEN_Node* from, TEN_Node* to)
{
  decrement(from, to);
  increment(to, from);
}

int LibTEN::ResidualNetwork::getFlowSum()
{
  return std::accumulate(source->children.begin(), source->children.end(), 0,
                         [&] (int acc, TEN_Node* p)
                         { return acc + getCapacity(p, source); });
}

void LibTEN::ResidualNetwork::FordFulkerson()
{
  while (true) {
    // depth first search
    std::unordered_map<std::string, bool> CLOSED;
    auto dfs = [&](auto&& self, TEN_Node* p) -> TEN_Node* {
      // update closed list
      CLOSED[p->name] = true;

      // reach goal
      if (p == sink) return p;

      // check children
      LibTEN::TEN_Nodes next;
      next.insert(next.end(), p->children.begin(), p->children.end());
      next.insert(next.end(), p->parents.begin(), p->parents.end());
      for (auto q : next) {

        // already searched
        if (CLOSED.find(q->name) != CLOSED.end()) continue;

        // fulfill
        if (getCapacity(p, q) == 0) continue;

        // call dfs recursively
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
