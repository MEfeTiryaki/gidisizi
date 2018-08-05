#pragma once

#include "gidisizi/RRT/RRT.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class RRTNonHolonomic: public RRT<NodeType,Environment>
{
 public:
  // Constructor.
  RRTNonHolonomic();
  // Destructor.
  virtual ~RRTNonHolonomic();


  virtual bool Plan() override;

protected:
  bool steer(NodeType* qNew, NodeType* qNear, NodeType& qRand) override;

};
}  // gidisizi
#include "gidisizi/examples/RRTNonHolonomic.tpp"
