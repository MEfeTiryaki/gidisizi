#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

#include <boost/thread/thread.hpp>

#include "gidisizi/utils/Node.hpp"
#include "gidisizi/utils/Graph.hpp"

namespace gidisizi {
  template<typename NodeType,typename Environment>
  class RRTBase
  {
   public:
    // Constructor.
    RRTBase();
    // Destructor.
    virtual ~RRTBase();
    // Init
    virtual void initilize(Environment* environment,NodeType* q_init, NodeType* q_goal, Eigen::VectorXd solutionAccuracy,
                           Eigen::VectorXd upper, Eigen::VectorXd lower, int maxIteration,
                           double maxTime);

    virtual void advance();

    virtual bool Plan();

    NodeType* getInit(){
      return qInit_;
    }
    NodeType* getGoal(){
      return qGoal_;
    }
    Eigen::VectorXd getSolutionAccuracy(){
      return solutionAccuracy_;
    }
    std::vector<NodeType*> getPath(){
      return path_;
    };
    std::vector<std::vector<NodeType*>> getDebugPaths(){
      return debugPaths_;
    };
    gidisizi::Graph<NodeType> getGraph(){
      return G_;
    };
    std::vector<gidisizi::Graph<NodeType>> getDebugGraphes(){
      return debugGraphes_;
    };

   protected:

    virtual void drawRandomConfiguration(NodeType& q);
    virtual bool steer(NodeType* qNew,NodeType* qNear,NodeType& qRand);

    bool isGoalReached(NodeType* qNew);
    bool isGoalReachable();
    void backTrackPath();
    std::vector<NodeType*> backTrackPath(NodeType* solution);

    NodeType* qInit_;
    NodeType* qGoal_;
    std::vector<NodeType*> solutionNodes_;
    NodeType* solutionNode_;
    Eigen::VectorXd solutionAccuracy_;
    Eigen::VectorXd upper_;
    Eigen::VectorXd lower_;

    std::vector<NodeType*> path_ ;
    gidisizi::Graph<NodeType> G_ ;
    std::vector<gidisizi::Graph<NodeType>> debugGraphes_ ;
    std::vector<std::vector<NodeType*>> debugPaths_ ;

    Environment* environment_;
    int maxIteration_;
    double maxTime_;

  };
}  // gidisizi
#include "gidisizi/RRT/RRTBase.tpp"
