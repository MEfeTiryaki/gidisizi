#include "gidisizi/Node.hpp"

namespace gidisizi {

template<int stateLength>
Node<stateLength>::Node()
    : Node(0)
{
}
template<int stateLength>
Node<stateLength>::Node(int id)
    : Node(id, Eigen::VectorXd(stateLength))
{
}

template<int stateLength>
Node<stateLength>::Node(int id, Eigen::VectorXd state)
    : id_(id),
      state_(state)
{
  parent_ = this ;
}
template<int stateLength>
Node<stateLength>::Node(int id, Eigen::VectorXd state, gidisizi::Node<stateLength>* parent)
    : id_(id),
      state_(state),
      parent_(parent)
{
  parent->addChild(this);
}

template<int stateLength>
Node<stateLength>::~Node()
{
}

// GETTER/SETTERs

template<int stateLength>
void Node<stateLength>::addChild(gidisizi::Node<stateLength>* child)
{
  this->children_.push_back(child);
  child->setParent(this);
}

template<int stateLength>
std::vector<gidisizi::Node<stateLength>*> Node<stateLength>::getChildren()
{
  return children_;
}

template<int stateLength>
gidisizi::Node<stateLength>* Node<stateLength>::getChild(int id)
{
  return children_[id];
}

template<int stateLength>
void Node<stateLength>::setParent(gidisizi::Node<stateLength>* parent)
{
  parent_ = parent;
}
template<int stateLength>
gidisizi::Node<stateLength>* Node<stateLength>::getParent()
{
  return parent_;
}

template<int stateLength>
Eigen::VectorXd Node<stateLength>::getState()
{
  return state_;
}

template<int stateLength>
void Node<stateLength>::setState(Eigen::VectorXd state)
{
  state_ = state;
}
template<int stateLength>
int Node<stateLength>::getId()
{
  return id_;
}

template<int stateLength>
void Node<stateLength>::setId(int id)
{
  id_ = id;
}

} /* namespace iyi*/
