#include "gidisizi/RRT.hpp"

namespace gidisizi {

template<typename NodeType>
RRT<NodeType>::RRT()
    : qInit_(),
      qGoal_(),
      maxTime_(10.0),
      maxIteration_(1000)
{
}

template<typename NodeType>
RRT<NodeType>::~RRT()
{
}

template<typename NodeType>
void RRT<NodeType>::initilize(NodeType* q_init, NodeType* q_goal, Eigen::VectorXd solutionAccuracy,
                              Eigen::VectorXd upper, Eigen::VectorXd lower, int maxIteration,
                              double maxTime)
{

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

template<typename NodeType>
void RRT<NodeType>::advance()
{

}

template<typename NodeType>
bool RRT<NodeType>::Plan()
{
  bool success = true;
  int nodeId = 2;
  int iter = 0;
  double startTime = clock();
  while (true) {
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
    G_.getNearestVertex(qNearest, qRandom);
    while (!steer(qNew, qNearest, qRandom, 1.0)) {
    }
    qNearest->addChild(qNew);
    G_.addVertex(qNew);
    G_.addEdge(qNearest, qNew);
    if (isGoalReached(qNew)) {
      solutionNode_ = qNew;
      break;
    }
    iter++;
  }
  backTrackPath();
  return success;
}

template<typename NodeType>
void RRT<NodeType>::backTrackPath()
{
  NodeType* cursor = solutionNode_;
  path_.push_back(cursor);
  while (cursor->getParent() != cursor) {
    cursor = cursor->getParent();
    path_.push_back(cursor);
  }
  std::reverse(std::begin(path_), std::end(path_));
}

template<typename NodeType>
void RRT<NodeType>::drawRandomConfiguration(NodeType& q)
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

template<typename NodeType>
bool RRT<NodeType>::steer(NodeType* qNew, NodeType* qNear, NodeType& qRand, double deltaQ)
{
  qNew->setState(qNear->getState() + deltaQ * (qRand.getState() - qNear->getState()));
  return true;
}

template<typename NodeType>
bool RRT<NodeType>::isGoalReached(NodeType* qNew)
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

template<typename NodeType>
bool RRT<NodeType>::isGoalReachable()
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

template<typename NodeType>
void RRT<NodeType>::drawGraph()
{
  int w = 800;
  int thickness = 2;
  int lineType = 8;
  cv::Scalar graphColor = cv::Scalar(255, 255, 255);
  cv::Scalar solution = cv::Scalar(0, 255, 0);
  cv::Scalar goalcolor = cv::Scalar(255, 0, 255);

  cv::Mat img = cv::Mat::zeros(w, w, CV_8UC3);

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

