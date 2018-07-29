#include "gidisizi/utils/Graph.hpp"

namespace gidisizi {
template<typename NodeType>
Graph<NodeType>::Graph()
{
  verteces_ = std::vector<NodeType*>(0);
  edges_ = std::vector< gidisizi::Edge<NodeType>> (0);
}
template<typename NodeType>
Graph<NodeType>::~Graph()
{
}

template<typename NodeType>
void Graph<NodeType>::initilize()
{

}

// GETTER/SETTERs
template<typename NodeType>
void Graph<NodeType>::addVertex(NodeType* v)
{
  verteces_.push_back(v);
}

template<typename NodeType>
void Graph<NodeType>::addEdge(NodeType* s, NodeType* e)
{
  gidisizi::Edge<NodeType> edge;
  edge.start = s;
  edge.end = e;
  addEdge(edge);
}
template<typename NodeType>
void Graph<NodeType>::addEdge(gidisizi::Edge<NodeType> edge)
{
  edges_.push_back(edge);
}

template<typename NodeType>
std::vector<gidisizi::Edge<NodeType>> Graph<NodeType>::getEdges()
{
  return edges_;
}

template<typename NodeType>
std::vector<NodeType*> Graph<NodeType>::getVerteces(){
  return verteces_;
}
template<typename NodeType>
void Graph<NodeType>::getNearestVertex(NodeType*& nearestVertex, NodeType& v_new)
{
  double distance = 1000.0;
  double newDistance = 0.0;
  for (auto v : verteces_) {
    newDistance = (v_new.getState() - v->getState()).squaredNorm();
    if (newDistance < distance) {
      nearestVertex = v;
      distance = newDistance;
    }
  }
}

} /* namespace iyi*/
