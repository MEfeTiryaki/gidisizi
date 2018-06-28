#include "gidisizi/Node.hpp"

namespace gidisizi {

Node::Node()
    : Node(0)
{
}
Node::Node(int id)
    : Node(id, Eigen::VectorXd())
{
}
Node::Node(int id, Eigen::VectorXd state)
    : id_(id),
      state_(state)
{
  parent_ = this ;
}
Node::Node(int id, Eigen::VectorXd state, gidisizi::Node* parent)
    : id_(id),
      state_(state),
      parent_(parent)
{
  parent->addchild(this);
}

Node::~Node()
{
}

// GETTER/SETTERs

void Node::addchild(gidisizi::Node* child)
{
  this->children_.push_back(child);
}

std::vector<gidisizi::Node*> Node::getchildren()
{
  return children_;
}

gidisizi::Node* Node::getChild(int id)
{
  return children_[id];
}

void Node::setParent(gidisizi::Node* parent)
{
  parent_ = parent;
}
gidisizi::Node* Node::getParent()
{
  return parent_;
}

Eigen::VectorXd Node::getState()
{
  return state_;
}

void Node::setState(Eigen::VectorXd state)
{
  state_ = state;
}
int Node::getId()
{
  return id_;
}

void Node::setId(int id)
{
  id_ = id;
}

} /* namespace iyi*/
