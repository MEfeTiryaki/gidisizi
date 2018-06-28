#pragma once

#include "gidisizi/Node.hpp"
#include <vector>
#include "Eigen/Dense"

namespace gidisizi {
class Node
{
 public:
  // Constructors
  Node();

  Node(int id);

  Node(int id, Eigen::VectorXd state);

  Node(int id, Eigen::VectorXd state, gidisizi::Node* parent);

  // Destructor.
  virtual ~Node();
  // Init
  virtual void initilize()
  {
  }
  ;

  virtual void addchild(gidisizi::Node* child);

  std::vector<gidisizi::Node*> getchildren();

  gidisizi::Node* getChild(int id);

  void setParent(gidisizi::Node* parent);

  gidisizi::Node* getParent();

  Eigen::VectorXd getState();

  void setState(Eigen::VectorXd state);

  int getId();

  void setId(int id);

 protected:
  int id_;
  Eigen::VectorXd state_;

  std::vector<gidisizi::Node*> children_;
  gidisizi::Node* parent_;
};
}  // gidisizi
