#pragma once

#include "gidisizi/RRT/RRT.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class StateSpacePlan: public RRT<NodeType,Environment>
{
 public:
  // Constructor.
  StateSpacePlan();
  // Destructor.
  virtual ~StateSpacePlan();


  virtual bool Plan() override;

protected:
  bool steer(NodeType* qNew, NodeType* qNear, NodeType& qRand) override;

};
}  // gidisizi
#include "gidisizi/examples/StateSpacePlan.tpp"
