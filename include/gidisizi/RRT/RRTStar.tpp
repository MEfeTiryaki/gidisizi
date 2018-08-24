#include "gidisizi/RRT/RRTStar.hpp"

namespace gidisizi {

template<typename NodeType, typename Environment>
RRTStar<NodeType, Environment>::RRTStar()
    : RRTBase<NodeType, Environment>()
{
}

template<typename NodeType, typename Environment>
RRTStar<NodeType, Environment>::~RRTStar()
{
}

template<typename NodeType, typename Environment>
bool RRTStar<NodeType, Environment>::Plan()
{
  bool success = true;
  int nodeId = 2;
  int iter = 0;
  double startTime = clock();
  double debugingTime = 0.0;
  bool foundSolution = false ;
  while (true) {
    //std::cerr<<"\t"<<iter<<std::endl;
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
    //std::cerr<<"random : "<<qRandom.getState().transpose()<<std::endl;

    this->G_.getNearestVertex(qNearest, qRandom);
    //std::cerr<<"nearest : "<<qNearest->getState().transpose()<<std::endl;

    if(steer(qNew, qNearest, qRandom)) {
      continue;
    }
    //std::cerr<<"steer : "<<qNew->getState().transpose()<<std::endl;

    if(!findNearNodes(qNew)){
      //std::cerr<<"No near node"<<std::endl;
      continue;
    }
    //std::cerr<<"number of close nodes : "<<qNew->getNumberOfCloseNodes()<<std::endl;

    qNearest = lowestCostNeighbor(qNew);
    qNearest->addChild(qNew);

    repairPaths(qNew);

    gidisizi::Line* edge = new gidisizi::Line(qNearest->getState(), qNew->getState());
    qNew->setPathToThis(edge);
    this->G_.addVertex(qNew);
    this->G_.addEdge(edge);

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
    }else{
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
        gidisizi::Line* edge = new gidisizi::Line(this->qInit_->getState(), this->qGoal_->getState());
        this->qGoal_->setPathToThis(edge);
      }

      debugingTime += (clock()-t);
    }
    iter++;
  }
  this->backTrackPath();
  std::cerr << "Planning duration : " << (clock() - startTime - debugingTime) / double(CLOCKS_PER_SEC)
            << std::endl;
  return success;
}


template<typename NodeType, typename Environment>
bool RRTStar<NodeType, Environment>::steer(NodeType* qNew, NodeType* qNear, NodeType& qRand)
{
  //double deltaQ = 0.5/(sqrt(qNew->getId()/10)+1)+0.5;
  int n = qNear->getState().size();
  Eigen::MatrixXd deltaQ = Eigen::MatrixXd::Zero(n,n);
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  deltaQ.diagonal() = Eigen::VectorXd::Ones(n)*distribution(generator);
  qNew->setState(qNear->getState() + deltaQ * (qRand.getState() - qNear->getState()));
  int trial = 1 ;
  while(trial++<5 && this->environment_->checkCollisions(qNear->getState(), qNew->getState())){
    //std::cerr<< trial << std::endl;
    qNew->setState(qNear->getState() + deltaQ/pow(2,trial) * (qRand.getState() - qNear->getState()));
  }
  return this->environment_->checkCollisions(qNear->getState(), qNew->getState());
}

template<typename NodeType, typename Environment>
bool RRTStar<NodeType, Environment>::findNearNodes(NodeType* qNew)
{
  double distance = 0.3/(sqrt(qNew->getId()/10)+1)+0.3;
  for (auto n : this->G_.getVerteces()) {
    if ((qNew->getState() - n->getState()).norm() < distance
        && !this->environment_->checkCollisions(n->getState(), qNew->getState())) {
      qNew->addCloseNode(n);
    }
  }
  return qNew->getNumberOfCloseNodes();
}

template<typename NodeType, typename Environment>
NodeType* RRTStar<NodeType, Environment>::lowestCostNeighbor(NodeType* qNew)
{
  NodeType* qNearest;
  double cost = 100000.0 ;
  double newCost = 1000.0 ;
  for (auto n : qNew->getCloseNodes()) {
    newCost = n->getCost()+ (qNew->getState() - n->getState()).norm();
    if (newCost < cost) {
      qNearest = n;
      cost = newCost;
    }
  }
  qNew->setCost(cost);
  return qNearest;
}


template<typename NodeType, typename Environment>
void RRTStar<NodeType, Environment>::repairPaths(NodeType* qNew)
{
  for (auto n : qNew->getCloseNodes()) {
    NodeType* oldParent = n;
    NodeType* newParent = qNew ;
    NodeType* tempParent;
    double newCost;
    while(true){
        newCost = oldParent->getCost() + (newParent->getState() - oldParent->getState()).norm();
        if(oldParent->getCost() > newCost){
          tempParent = oldParent->getParent();
          oldParent->setParent(newParent);
          oldParent->setCost(newCost);
          newParent = oldParent;
          oldParent = tempParent;
        }else{
          break;
        }
    }
  }
}

}
/* namespace gidisizi*/
