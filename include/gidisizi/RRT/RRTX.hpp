#pragma once

#include "gidisizi/RRT/RRTBase.hpp"
#include "gidisizi/utils/Line.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class RRTX:public RRTBase< NodeType, Environment>
{
 public:
  // Constructor.
  RRTX();
  // Destructor.
  virtual ~RRTX();

  virtual bool Plan();

 protected:

  virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand);
  virtual bool findNearNodes(NodeType* qNew);
  virtual NodeType* lowestCostNeighbor(NodeType* qNew);
  void repairPaths(NodeType* qNew);
  void repairWire();

};
}  // gidisizi
#include "gidisizi/RRT/RRTX.tpp"
