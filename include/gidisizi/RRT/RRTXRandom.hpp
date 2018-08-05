#pragma once

#include "gidisizi/RRT/RRTBase.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class RRTXRandom:public RRTBase< NodeType, Environment>
{
 public:
  // Constructor.
  RRTXRandom();
  // Destructor.
  virtual ~RRTXRandom();

  virtual bool Plan();

 protected:

  virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand);
  virtual bool findNearNodes(NodeType* qNew);
  virtual NodeType* lowestCostNeighbor(NodeType* qNew);
  void repairPaths(NodeType* qNew);
  void repairWire();

};
}  // gidisizi
#include "gidisizi/RRT/RRTXRandom.tpp"
