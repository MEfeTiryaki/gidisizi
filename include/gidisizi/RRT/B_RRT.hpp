#pragma once

#include "gidisizi/RRT/RRT.hpp"
#include "gidisizi/utils/Bezier3.hpp"

namespace gidisizi {
template<typename NodeType,typename Environment>
class B_RRT: public RRT<NodeType,Environment>
{
 public:
  // Constructor.
  B_RRT();
  // Destructor.
  virtual ~B_RRT();

  virtual bool Plan();

protected:
  bool steer(NodeType* qNew, NodeType* qNear, NodeType& qRand) override;

    virtual void connectNewNode(NodeType* qNew, NodeType* qNearest) override;
};
}  // gidisizi
#include "gidisizi/RRT/B_RRT.tpp"
