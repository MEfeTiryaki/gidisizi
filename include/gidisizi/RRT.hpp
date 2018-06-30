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

namespace gidisizi {
template<typename NodeType>
class RRT
{
 public:
  // Constructor.
  RRT();
  // Destructor.
  virtual ~RRT();
  // Init
  virtual void initilize(NodeType* q_init, NodeType* q_goal, Eigen::VectorXd solutionAccuracy,
                         Eigen::VectorXd upper, Eigen::VectorXd lower, int maxIteration,
                         double maxTime);

  virtual void advance();

  virtual bool Plan();

  virtual void drawGraph();

 protected:

  virtual void drawRandomConfiguration(NodeType& q);
  virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand,double deltaQ);
  virtual bool isGoalReached(NodeType* qNew);
  virtual bool isGoalReachable();
  virtual void backTrackPath();

  NodeType* qInit_;
  NodeType* qGoal_;
  NodeType* solutionNode_;
  std::vector<NodeType*> path_ ;
  Eigen::VectorXd solutionAccuracy_;
  Eigen::VectorXd upper_;
  Eigen::VectorXd lower_;
  gidisizi::Graph<NodeType> G_ ;

  int maxIteration_;
  double maxTime_;

};
}  // gidisizi
#include "gidisizi/RRT.tpp"
