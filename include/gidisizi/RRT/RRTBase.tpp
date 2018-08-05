#include "gidisizi/RRT/RRTBase.hpp"

namespace gidisizi {

  template<typename NodeType, typename Environment>
  RRTBase<NodeType, Environment>::RRTBase()
      : environment_(),
        qInit_(),
        qGoal_(),
        maxTime_(10.0),
        maxIteration_(1000),
        solutionNode_()
  {
  }

  template<typename NodeType, typename Environment>
  RRTBase<NodeType, Environment>::~RRTBase()
  {
  }

  template<typename NodeType, typename Environment>
  void RRTBase<NodeType, Environment>::initilize(Environment* environment, NodeType* q_init,
                                              NodeType* q_goal, Eigen::VectorXd solutionAccuracy,
                                              Eigen::VectorXd upper, Eigen::VectorXd lower,
                                              int maxIteration, double maxTime)
  {
    environment_ = environment;
    qInit_ = q_init;
    qGoal_ = q_goal;
    solutionAccuracy_ = solutionAccuracy;
    upper_ = upper;
    lower_ = lower;
    if (!isGoalReachable()) {
      std::cerr << "Goal is not in constrains!!!" << std::endl;
    }
    maxIteration_ = maxIteration;
    maxTime_ = maxTime;
    G_ = gidisizi::Graph<NodeType>();
    G_.addVertex(qInit_);
  }

  template<typename NodeType, typename Environment>
  void RRTBase<NodeType, Environment>::advance()
  {

  }

  template<typename NodeType, typename Environment>
  bool RRTBase<NodeType, Environment>::Plan()
  {
    return true;
  }

  template<typename NodeType, typename Environment>
  void RRTBase<NodeType, Environment>::backTrackPath()
  {
    NodeType* cursor = solutionNode_;
    double cost = 1000000.0;
    for (auto node : solutionNodes_) {
      if (node->getCost() < cost) {
        cost = node->getCost();
        cursor = node;
      }
    }
    path_.push_back(cursor);
    while (cursor->getParent() != cursor) {
      cursor = cursor->getParent();
      path_.push_back(cursor);
    }
    std::reverse(std::begin(path_), std::end(path_));
  }

  template<typename NodeType, typename Environment>
  std::vector<NodeType*> RRTBase<NodeType, Environment>::backTrackPath(NodeType* solution)
  {
    std::vector<NodeType*> path;
    NodeType* cursor = solution;
    double cost = 1000000.0;
    for (auto node : solutionNodes_) {
      if (node->getCost() < cost) {
        cost = node->getCost();
        cursor = node;
      }
    }
    path.push_back(cursor);
    while (cursor->getParent() != cursor) {
      cursor = cursor->getParent();
      path.push_back(cursor);
    }
    std::reverse(std::begin(path), std::end(path));
    return path;
  }


  template<typename NodeType, typename Environment>
  void RRTBase<NodeType, Environment>::drawRandomConfiguration(NodeType& q)
  {
    Eigen::VectorXd mid = (upper_ + lower_) / 2;
    Eigen::VectorXd length = (upper_ - lower_) / 2;
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(-1.0, 1.0);
    Eigen::VectorXd conf = Eigen::VectorXd(q.getState().size());
    for (int i = 0; i < conf.size(); i++) {
      double randnum = distribution(generator);
      conf[i] = randnum * length[i] + mid[i];
    }
    q.setState(conf);
  }

  template<typename NodeType, typename Environment>
  bool RRTBase<NodeType, Environment>::steer(NodeType* qNew, NodeType* qNear, NodeType& qRand)
  {
    double deltaQ = 1.0;
    qNew->setState(qNear->getState() + deltaQ * (qRand.getState() - qNear->getState()));
    return environment_->checkCollisions(qNear->getState(),qNew->getState());
  }

  template<typename NodeType, typename Environment>
  bool RRTBase<NodeType, Environment>::isGoalReached(NodeType* qNew)
  {
    Eigen::VectorXd state = qNew->getState();
    Eigen::VectorXd goalState = qGoal_->getState();
    for (int i = 0; i < state.size(); i++) {
      if (state[i] > goalState[i] + solutionAccuracy_[i]) {
        return false;
      } else if (state[i] < goalState[i] - solutionAccuracy_[i]) {
        return false;
      }
    }
    return true;
  }

  template<typename NodeType, typename Environment>
  bool RRTBase<NodeType, Environment>::isGoalReachable()
  {
    Eigen::VectorXd goalState = qGoal_->getState();
    for (int i = 0; i < goalState.size(); i++) {
      if (goalState[i] > upper_[i]) {
        return false;
      } else if (goalState[i] < lower_[i]) {
        return false;
      }
    }
    return true;
  }

}
/* namespace iyi*/
