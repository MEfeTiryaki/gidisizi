#include "gidisizi/RRT/RRTX.hpp"

namespace gidisizi {

template<typename NodeType, typename Environment>
RRTX<NodeType, Environment>::RRTX()
    : RRTBase<NodeType, Environment>()
{
}

template<typename NodeType, typename Environment>
RRTX<NodeType, Environment>::~RRTX()
{
}

template<typename NodeType, typename Environment>
bool RRTX<NodeType, Environment>::Plan()
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

    if(!findNearNodes(qNew)) {
      //std::cerr<<"No near node"<<std::endl;
      continue;
    }
    //std::cerr<<"number of close nodes : "<<qNew->getNumberOfCloseNodes()<<std::endl;

    qNearest = lowestCostNeighbor(qNew);
    qNearest->addChild(qNew);

    repairPaths(qNew);

    this->G_.addVertex(qNew);
    this->G_.addEdge(qNearest, qNew);

    repairWire();

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
bool RRTX<NodeType, Environment>::steer(NodeType* qNew, NodeType* qNear, NodeType& qRand)
{
  double deltaQ = 0.2/(sqrt(qNew->getId()/10)+1)+0.2;
  Eigen::VectorXd newState = qNear->getState() + deltaQ * (qRand.getState() - qNear->getState()).normalized();
  for(int i = 0 ; i< newState.size();i++){
    if(newState[i]>this->upper_[i]){
      newState[i]= this->upper_[i];
    }
    if(newState[i]<this->lower_[i]){
      newState[i]= this->lower_[i];
    }
  }
  qNew->setState(newState);
  return this->environment_->checkCollisions(qNear->getState(), qNew->getState());
}

template<typename NodeType, typename Environment>
bool RRTX<NodeType, Environment>::findNearNodes(NodeType* qNew)
{
  double distance = 0.4/(sqrt(qNew->getId()/10)+1)+0.4;
  for (auto n : this->G_.getVerteces()) {
    if ((qNew->getState() - n->getState()).norm() < distance
        && !this->environment_->checkCollisions(n->getState(), qNew->getState())) {
      qNew->addCloseNode(n);
      // XXX: also add to closest nodes of the already existent node
      n->addCloseNode(qNew);
    }
  }
  return qNew->getNumberOfCloseNodes();
}

template<typename NodeType, typename Environment>
NodeType* RRTX<NodeType, Environment>::lowestCostNeighbor(NodeType* qNew)
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
void RRTX<NodeType, Environment>::repairPaths(NodeType* qNew)
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

template<typename NodeType, typename Environment>
void RRTX<NodeType, Environment>::repairWire()
{
  double newCost;
  for (auto vertex : this->G_.getVerteces()) {
    for (auto parrentCandidate : vertex->getCloseNodes()) {
      double newCost = parrentCandidate->getCost() + (vertex->getState() - parrentCandidate->getState()).norm();
      if(vertex->getCost() > newCost){
        vertex->setParent(parrentCandidate);
        vertex->setCost(newCost);
      }
    }
  }
}

}
/* namespace gidisizi*/
