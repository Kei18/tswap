#include "../include/lib_time_expanded_network.hpp"


std::unordered_map<std::string, TEN_Node*> TEN_Node::all_nodes;

TEN_Node::TEN_Node(NodeType _type, Node* _v, Node* _u, int _t)
  : type(_type), v(_v), u(_u), t(_t)
{
  name = getName(type, _v, _u, t);
}

void TEN_Node::addParent(TEN_Node* parent)
{
  parents.push_back(parent);
  parent->children.push_back(this);
}

void TEN_Node::removeParent(TEN_Node* parent)
{
  auto itr1 = std::find(parents.begin(), parents.end(), parent);
  if (itr1 == parents.end()) return;
  parents.erase(itr1);
  auto itr2 = std::find(parent->children.begin(), parent->children.end(), this);
  parent->children.erase(itr2);
}

TEN_Node* TEN_Node::createNewNode(NodeType _type, Node* _v, Node* _u, int _t)
{
  TEN_Node* new_node = new TEN_Node(_type, _v, _u, _t);
  all_nodes[new_node->name] = new_node;
  return new_node;
}

TEN_Node* TEN_Node::createNewNode(NodeType _type, Node* _v, int _t)
{
  return createNewNode(_type, _v, nullptr, _t);
}

// used for sink or source
TEN_Node* TEN_Node::createNewNode(NodeType _type)
{
  if ((_type != NodeType::SOURCE) && (_type != NodeType::SINK)) halt("invalid type");
  return createNewNode(_type, nullptr, nullptr, 0);
}

std::string TEN_Node::getName(NodeType _type, Node* _v, Node* _u, int _t)
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
    if (_u == nullptr) halt("invalid operation, TEN_Node::getName");
    return
      std::to_string(_v->id) + "_" +
      std::to_string(_u->id) + "_" +
      std::to_string(_t) + "_" +
      std::to_string(_type);
  default:
    halt("invalid operation, unknown TEN_Node type");
    return "";
  }
}

std::string TEN_Node::getName(NodeType _type, Node* _v, int _t)
{
  return getName(_type, _v, nullptr, _t);
}

std::string TEN_Node::getEdgeName(TEN_Node* p, TEN_Node* q)
{
  return p->name + "__" + q->name;
}

TEN_Node* TEN_Node::getNode(NodeType _type, Node* _v, Node* _u, int _t)
{
  auto itr = all_nodes.find(getName(_type, _v, _u, _t));
  return (itr != all_nodes.end()) ? itr->second : nullptr;
}

TEN_Node* TEN_Node::getNode(NodeType _type, Node* _v, int _t)
{
  return getNode(_type, _v, nullptr, _t);
}
