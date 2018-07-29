#pragma once

#include "gidisizi/Node.hpp"
#include <vector>
#include "Eigen/Dense"

#include <iostream>
namespace gidisizi {
template<typename NodeType>
struct Edge
{
  NodeType* start;
  NodeType* end;
};
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

  virtual void addEdge(NodeType* s,NodeType* e);
  virtual void addEdge(gidisizi::Edge<NodeType> edge);
  virtual std::vector<gidisizi::Edge<NodeType>> getEdges();

  virtual std::vector<NodeType*> getVerteces();

  virtual void getNearestVertex(NodeType*& qNear,NodeType& v);

 protected:
  std::vector<NodeType*> verteces_;
  std::vector<gidisizi::Edge<NodeType>> edges_;

};
}  // gidisizi

#include "gidisizi/Graph.tpp"
