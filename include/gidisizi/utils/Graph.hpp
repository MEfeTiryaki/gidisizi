#pragma once

#include "gidisizi/utils/Node.hpp"
#include <vector>
#include "Eigen/Dense"

#include <iostream>
namespace gidisizi {

template<typename NodeType>
class Graph
{
 public:
  // Constructors
  Graph();

  // Destructor.
  virtual ~Graph();
  // Init
  virtual void initilize();

  virtual void addVertex(NodeType* v);

  virtual void addEdge(gidisizi::Curve* edge);
  virtual std::vector<gidisizi::Curve*> getEdges();

  virtual std::vector<NodeType*> getVerteces();

  virtual void getNearestVertex(NodeType*& qNear,NodeType& v);

 protected:
  std::vector<NodeType*> verteces_;
  std::vector<gidisizi::Curve*> edges_;

};
}  // gidisizi

#include "gidisizi/utils/Graph.tpp"
