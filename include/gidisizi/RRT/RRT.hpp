#pragma once

#include "gidisizi/RRT/RRTBase.hpp"
#include "gidisizi/utils/Line.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class RRT: public RRTBase<NodeType,Environment>
{
 public:
  // Constructor.
  RRT();
  // Destructor.
  virtual ~RRT();
  // Init
  virtual void advance();

  virtual bool Plan();

protected:
  bool steer(NodeType* qNew, NodeType* qNear, NodeType& qRand) override;


  virtual void connectNewNode(NodeType* qNew, NodeType* qNearest);
};
}  // gidisizi
#include "gidisizi/RRT/RRT.tpp"
