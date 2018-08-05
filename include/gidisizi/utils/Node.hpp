#pragma once
#include <vector>
#include "Eigen/Dense"

namespace gidisizi {

template<int stateLength>
class Node
{
 public:
  // Constructors
  Node();

  Node(int id);

  Node(int id, Eigen::VectorXd state);

  Node(int id, Eigen::VectorXd state, gidisizi::Node<stateLength>* parent);

  // Destructor.
  virtual ~Node();
  // Init
  virtual void initilize()
  {
  }
  ;

  virtual void addChild(gidisizi::Node<stateLength>* child);

  std::vector<gidisizi::Node<stateLength>*> getChildren();

  gidisizi::Node<stateLength>* getChild(int id);

  void setParent(gidisizi::Node<stateLength>* parent);

  gidisizi::Node<stateLength>* getParent();

  Eigen::VectorXd getState();

  void setState(Eigen::VectorXd state);

  int getId();

  void setId(int id);

  void setCost(double cost) ;

  double getCost();

  void addCloseNode(gidisizi::Node<stateLength>* node);
  std::vector<gidisizi::Node<stateLength>*> getCloseNodes();
  int getNumberOfCloseNodes();

  void addLessCloseNode(gidisizi::Node<stateLength>* node);
  std::vector<gidisizi::Node<stateLength>*> getLessCloseNodes();
  int getNumberOfLessCloseNodes();
  
 protected:
  int id_;
  Eigen::VectorXd state_;

  double cost_;

  std::vector<gidisizi::Node<stateLength>*> children_;
  gidisizi::Node<stateLength>* parent_;

  std::vector<gidisizi::Node<stateLength>*> closeNodes_;
  std::vector<gidisizi::Node<stateLength>*> lessCloseNodes_;

};
}  // gidisizi
#include "gidisizi/utils/Node.tpp"
