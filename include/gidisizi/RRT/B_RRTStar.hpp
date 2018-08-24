#pragma once

#include "gidisizi/RRT/RRTStar.hpp"
#include "gidisizi/utils/Line.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class B_RRTStar : public RRTStar<NodeType,Environment>
{
 public:
  // Constructor.
  B_RRTStar();
  // Destructor.
  virtual ~B_RRTStar();

  virtual bool Plan() override;

 protected:
  virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand);
  virtual bool findNearNodes(NodeType* qNew);
  virtual NodeType* lowestCostNeighbor(NodeType* qNew);
  void repairPaths(NodeType* qNew);

};
}  // gidisizi
#include "gidisizi/RRT/B_RRTStar.tpp"
