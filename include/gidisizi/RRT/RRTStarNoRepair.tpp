#include "gidisizi/RRTStarNoRepair.hpp"

namespace gidisizi {

template<typename NodeType, typename Environment>
RRTStarNoRepair<NodeType, Environment>::RRTStarNoRepair()
    : environment_(),
      qInit_(),
      qGoal_(),
      maxTime_(10.0),
      maxIteration_(1000),
      solutionNode_()
{
}

template<typename NodeType, typename Environment>
RRTStarNoRepair<NodeType, Environment>::~RRTStarNoRepair()
{
}

template<typename NodeType, typename Environment>
void RRTStarNoRepair<NodeType, Environment>::initilize(Environment* environment, NodeType* q_init,
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
void RRTStarNoRepair<NodeType, Environment>::advance()
{

}

template<typename NodeType, typename Environment>
bool RRTStarNoRepair<NodeType, Environment>::Plan()
{
  bool success = true;
  int nodeId = 2;
  int iter = 0;
  double startTime = clock();
  while (true) {
    std::cerr<<"\t"<<iter<<std::endl;
    if (!(iter < maxIteration_)) {
      std::cout << "Planner exceeded maximum iteration number." << std::endl;
      break;
    }
    if (!((clock() - startTime) / double(CLOCKS_PER_SEC) < maxTime_)) {
      std::cout << "Planner exceeded maximum computation time." << std::endl;
      break;
    }

    NodeType qRandom = NodeType();
    //qRandom.setState(Eigen::VectorXd::Zero(2));
    NodeType* qNearest;
    NodeType* qNew = new NodeType(nodeId++, Eigen::VectorXd::Zero(2));

    drawRandomConfiguration(qRandom);
    //std::cerr<<"random : "<<qRandom.getState().transpose()<<std::endl;

    G_.getNearestVertex(qNearest, qRandom);
    //std::cerr<<"nearest : "<<qNearest->getState().transpose()<<std::endl;

    if(steer(qNew, qNearest, qRandom, 0.2)) {
      continue;
    }
    //std::cerr<<"steer : "<<qNew->getState().transpose()<<std::endl;

    if(!findNearNodes(qNew,0.5)){
      //std::cerr<<"No near node"<<std::endl;
      continue;
    }
    //std::cerr<<"number of close nodes : "<<qNew->getNumberOfCloseNodes()<<std::endl;

    lowestCostNeighbor(qNew)->addChild(qNew);

    G_.addVertex(qNew);
    G_.addEdge(qNearest, qNew);
    if (isGoalReached(qNew)) {
      solutionNode_ = qNew;
      break;
    }
    iter++;
  }
  backTrackPath();
  std::cerr << "Planning duration : " << (clock() - startTime) / double(CLOCKS_PER_SEC)
            << std::endl;
  return success;
}

template<typename NodeType, typename Environment>
void RRTStarNoRepair<NodeType, Environment>::backTrackPath()
{
  NodeType* cursor = solutionNode_;
  path_.push_back(cursor);
  while (cursor->getParent() != cursor) {
    cursor = cursor->getParent();
    path_.push_back(cursor);
  }
  std::reverse(std::begin(path_), std::end(path_));
}

template<typename NodeType, typename Environment>
void RRTStarNoRepair<NodeType, Environment>::drawRandomConfiguration(NodeType& q)
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
bool RRTStarNoRepair<NodeType, Environment>::steer(NodeType* qNew, NodeType* qNear, NodeType& qRand,
                                           double deltaQ)
{
  qNew->setState(qNear->getState() + deltaQ * (qRand.getState() - qNear->getState()));
  return environment_->checkCollisions(qNear->getState(), qNew->getState());
}

template<typename NodeType, typename Environment>
bool RRTStarNoRepair<NodeType, Environment>::findNearNodes(NodeType* qNew, double distance)
{
  for (auto n : G_.getVerteces()) {
    //std::cerr<<"distance : "<<(qNew->getState() - n->getState()).squaredNorm()<<std::endl;
    //std::cerr<<"collision : "<<environment_->checkCollisions(n->getState(), qNew->getState())<<std::endl;

    if ((qNew->getState() - n->getState()).squaredNorm() < distance
        && !environment_->checkCollisions(n->getState(), qNew->getState())) {
      //std::cerr<<"node added"<<std::endl;
      qNew->addCloseNode(n);
    }
  }

  return qNew->getNumberOfCloseNodes();
}

template<typename NodeType, typename Environment>
NodeType* RRTStarNoRepair<NodeType, Environment>::lowestCostNeighbor(NodeType* qNew)
{
  NodeType* qNearest;
  double cost = 100000.0 ;
  double newCost = 1000.0 ;
  for (auto n : qNew->getCloseNodes()) {
    newCost = n->getCost()+ (qNew->getState() - n->getState()).squaredNorm();
    if (newCost < cost) {
      qNearest = n;
      cost = newCost;
    }
  }
  qNew->setCost(cost);
  return qNearest;
}

template<typename NodeType, typename Environment>
bool RRTStarNoRepair<NodeType, Environment>::isGoalReached(NodeType* qNew)
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
bool RRTStarNoRepair<NodeType, Environment>::isGoalReachable()
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

template<typename NodeType, typename Environment>
void RRTStarNoRepair<NodeType, Environment>::drawGraph()
{
  int w = 800;
  int thickness = 2;
  int lineType = 8;
  cv::Scalar graphColor = cv::Scalar(255, 255, 255);
  cv::Scalar solution = cv::Scalar(0, 255, 0);
  cv::Scalar goalcolor = cv::Scalar(255, 0, 255);
  cv::Scalar wallcolor = cv::Scalar(0, 0, 255);

  cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);

  for (auto wall : environment_->getWalls()) {
    cv::rectangle(img, cv::Point(w / 2 * (1 + (wall.point1[0])), w / 2 * (1 - (wall.point1[1]))),
                  cv::Point(w / 2 * (1 + (wall.point2[0])), w / 2 * (1 - (wall.point2[1]))),
                  wallcolor);
  }
  // Goal
  cv::rectangle(
      img,
      cv::Point(w / 2 * (1 + (qGoal_->getState()[0] - solutionAccuracy_[0])),
                w / 2 * (1 - (qGoal_->getState()[1] - solutionAccuracy_[1]))),
      cv::Point(w / 2 * (1 + (qGoal_->getState()[0] + solutionAccuracy_[0])),
                w / 2 * (1 - (qGoal_->getState()[1] + solutionAccuracy_[1]))),
      goalcolor);
  cv::imshow("Display window", img);
  cv::waitKey(1);
  for (auto e : G_.getEdges()) {
    cv::line(img,
             cv::Point(w / 2 * (1 + e.start->getState()[0]), w / 2 * (1 - e.start->getState()[1])),
             cv::Point(w / 2 * (1 + e.end->getState()[0]), w / 2 * (1 - e.end->getState()[1])),
             graphColor, thickness, lineType);
    cv::imshow("Display window", img);
    cv::waitKey(1);
  }
  for (int i = 0; i < path_.size() - 1; i++) {
    auto start = path_[i];
    auto end = path_[i + 1];
    cv::line(img, cv::Point(w / 2 * (1 + start->getState()[0]), w / 2 * (1 - start->getState()[1])),
             cv::Point(w / 2 * (1 + end->getState()[0]), w / 2 * (1 - end->getState()[1])),
             solution, thickness * 2, lineType);
    cv::imshow("Display window", img);
    cv::waitKey(1);
  }

  cv::waitKey(0);
}

}
/* namespace iyi*/
