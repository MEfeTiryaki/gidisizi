#include "gidisizi/examples/RRTNonHolonomic.hpp"

namespace gidisizi {

template<typename NodeType, typename Environment>
RRTNonHolonomic<NodeType, Environment>::RRTNonHolonomic()
    : RRT<NodeType, Environment>()
{
}

template<typename NodeType, typename Environment>
RRTNonHolonomic<NodeType, Environment>::~RRTNonHolonomic()
{
}
template<typename NodeType, typename Environment>
bool RRTNonHolonomic<NodeType, Environment>::Plan()
{
  bool success = true;
  int nodeId = 2;
  int iter = 0;
  double startTime = clock();
  double debugingTime = 0.0;
  bool foundSolution = false ;
  while (true) {
    if (!(iter < this->maxIteration_)) {
      std::cout << "Planner exceeded maximum iteration number." << std::endl;
      break;
    }
    if (!((clock() - startTime - debugingTime) / double(CLOCKS_PER_SEC) < this->maxTime_)) {
      std::cout << "Planner exceeded maximum computation time." << std::endl;
      break;
    }

    NodeType qRandom = NodeType();
    //qRandom.setState(Eigen::VectorXd::Zero(2));
    NodeType* qNearest;
    NodeType* qNew = new NodeType(iter);

    this->drawRandomConfiguration(qRandom);

    this->G_.getNearestVertex(qNearest, qRandom);

    if(steer(qNew, qNearest, qRandom)) {
      continue;
    }

    this->connectNewNode(qNew, qNearest);

    this->G_.addVertex(qNew);
    this->G_.addEdge(qNearest, qNew);
    // Debug
    double t = clock();
    this->debugGraphes_.push_back(this->G_);
    debugingTime += (clock()-t);
    if (this->isGoalReached(qNew)) {
      this->solutionNodes_.push_back(qNew);
      std::cerr << "iter : " << std::setw(5) <<iter
                << "  solution number : "<< std::setw(3) << this->solutionNodes_.size()
                << "  cost : " << std::setw(7) << this->solutionNodes_.back()->getCost()
                << "  time : " << std::setw(7) << (clock() - startTime - debugingTime) / double(CLOCKS_PER_SEC)
                << std::endl;
      // Debug
      double t = clock();
      this->debugPaths_.push_back(this->backTrackPath(qNew));
      debugingTime += (clock()-t);
      foundSolution = true;
    }
    else{
      // Debug
      double t = clock();
      if(foundSolution){
        this->debugPaths_.push_back(this->debugPaths_.back());
      }
      else{
        std::vector<NodeType*> emptyPath;
        emptyPath.push_back(this->qInit_);
        emptyPath.push_back(this->qGoal_);
        this->debugPaths_.push_back(emptyPath);
      }
      debugingTime += (clock()-t);
    }
    iter++;
  }
  std::cerr<<"Planning duration : "<<(clock() - startTime - debugingTime) /
  double(CLOCKS_PER_SEC)<<std::endl;
  this->backTrackPath();
  std::cerr<<"Path is generated!!"<<std::endl;
  return success;
}

template<typename NodeType, typename Environment>
bool RRTNonHolonomic<NodeType, Environment>::steer(NodeType* qNew, NodeType* qNear, NodeType& qRand)
{
  //double deltaQ = 0.4/(sqrt(qNew->getId()/20)+1)+0.3;
  int n = qNear->getState().size();
  Eigen::MatrixXd deltaQ = Eigen::MatrixXd::Zero(n,n);
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  deltaQ.diagonal() = Eigen::VectorXd::Ones(n)*distribution(generator);
  qNew->setState(qNear->getState() + deltaQ * (qRand.getState() - qNear->getState()));
  int trial = 1 ;
  while(trial++<5 && this->environment_->checkCollisions(qNear->getState(), qNew->getState())){
    qNew->setState(qNear->getState() + deltaQ/pow(2,trial) * (qRand.getState() - qNear->getState()));
  }
  return this->environment_->checkCollisions(qNear->getState(),qNew->getState());
}

}
/* namespace gidisizi*/
