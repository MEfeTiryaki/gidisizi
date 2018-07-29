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
template<typename NodeType,typename Environment>
class RRTStarNoRepair
{
 public:
  // Constructor.
  RRTStarNoRepair();
  // Destructor.
  virtual ~RRTStarNoRepair();
  // Init
  virtual void initilize(Environment* environment,NodeType* q_init, NodeType* q_goal, Eigen::VectorXd solutionAccuracy,
                         Eigen::VectorXd upper, Eigen::VectorXd lower, int maxIteration,
                         double maxTime);

  virtual void advance();

  virtual bool Plan();

  virtual void drawGraph();
  virtual NodeType* getInit(){
    return qInit_;
  }
  virtual NodeType* getGoal(){
    return qGoal_;
  }
  virtual Eigen::VectorXd getSolutionAccuracy(){
    return solutionAccuracy_;
  }
  virtual std::vector<NodeType*> getPath(){
    return path_;
  };
  virtual std::vector<std::vector<NodeType*>> getDebugPaths(){
    return debugPaths_;
  };
  virtual gidisizi::Graph<NodeType> getGraph(){
    return G_;
  };
  virtual std::vector<gidisizi::Graph<NodeType>> getDebugGraphes(){
    return debugGraphes_;
  };
 protected:

  virtual void drawRandomConfiguration(NodeType& q);
  virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand,double deltaQ);
  virtual bool isGoalReached(NodeType* qNew);
  virtual bool isGoalReachable();
  virtual void backTrackPath();

  virtual bool findNearNodes(NodeType* qNew, double distance);
  virtual NodeType* lowestCostNeighbor(NodeType* qNew);
  NodeType* qInit_;
  NodeType* qGoal_;
  NodeType* solutionNode_;
  std::vector<NodeType*> path_ ;
  Eigen::VectorXd solutionAccuracy_;
  Eigen::VectorXd upper_;
  Eigen::VectorXd lower_;
  gidisizi::Graph<NodeType> G_ ;

  Environment* environment_;
  int maxIteration_;
  double maxTime_;

};
}  // gidisizi
#include "gidisizi/RRTStarNoRepair.tpp"
