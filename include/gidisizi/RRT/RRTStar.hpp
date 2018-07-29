#pragma once

#include "gidisizi/RRT/RRTBase.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class RRTStar:public RRTBase<NodeType,Environment>
{
 public:
  // Constructor.
  RRTStar();
  // Destructor.
  virtual ~RRTStar();

  virtual bool Plan() override;

 protected:
  virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand);
  virtual bool findNearNodes(NodeType* qNew);
  virtual NodeType* lowestCostNeighbor(NodeType* qNew);
  void repairPaths(NodeType* qNew);

};
}  // gidisizi
#include "gidisizi/RRT/RRTStar.tpp"
