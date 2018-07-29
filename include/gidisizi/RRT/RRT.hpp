#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

#include <boost/thread/thread.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "gidisizi/Node.hpp"
#include "gidisizi/Graph.hpp"

#include "gidisizi/RRT/RRTBase.hpp"

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
  bool steer(NodeType* qNew, NodeType* qNear, NodeType& qRand,
                                          double deltaQ) override;


  virtual void connectNewNode(NodeType* qNew, NodeType* qNearest);
};
}  // gidisizi
#include "gidisizi/RRT/RRT.tpp"
