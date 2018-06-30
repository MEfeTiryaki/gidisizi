#include "ros/ros.h"
#include <iostream>
#include "gidisizi/Node.hpp"
#include "gidisizi/RRT.hpp"

int counter = 0;
using NodeType_ = gidisizi::Node<2>;
void testNode()
{
  NodeType_* node_1 = new NodeType_();
  NodeType_* node_2 = new NodeType_(2);
  NodeType_* node_3 = new NodeType_(3, Eigen::VectorXd::Ones(2) * 5);
  NodeType_* node_4 = new NodeType_(4, Eigen::VectorXd::Ones(2) * 4, node_3);
  NodeType_* node_5 = new NodeType_(5, Eigen::VectorXd::Ones(2) * 2, node_3);
  if (node_1->getId() != 0) {
    std::cerr << "1 - Node() constructor: id assignment not working properly" << std::endl;
  }
  if (node_2->getId() != 2) {
    std::cerr << "2 - Node(int) constructor: id assignment not working properly" << std::endl;
  }

  if (node_3->getId() != 3) {
    std::cerr << "3 - Node(int,Eigen::VectorXd) constructor: id assignment not working properly"
              << std::endl;
  }
  if (node_3->getState().size() != 5 && node_3->getState()[0] == 5) {
    std::cerr << "4 - Node(int,Eigen::VectorXd) constructor: state assignment not working properly"
              << std::endl;
  }

  if (node_4->getId() != 4) {
    std::cerr
        << "5 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: id assignment not working properly"
        << std::endl;
  }
  if (node_4->getState().size() != 5 && node_4->getState()[0] == 4) {
    std::cerr
        << "6 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: state assignment not working properly"
        << std::endl;
  }
  if (node_4->getParent() != node_3) {
    std::cerr
        << "7 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: parent assignment not working properly"
        << std::endl;
  }
  if (node_5->getParent() != node_3) {
    std::cerr
        << "8 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: parent assignment not working properly"
        << std::endl;
  }
  if (node_4 != node_3->getChild(0)) {
    std::cerr
        << "9 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: parent assignment not working properly"
        << std::endl;
  }
  if (node_5 != node_3->getChild(1)) {
    std::cerr
        << "10 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: parent assignment not working properly"
        << std::endl;
  }
  if (node_3->getChildren().size() != 2) {
    std::cerr
        << "11 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: parent assignment not working properly"
        << std::endl;
  }
}

void testRRT(){
  gidisizi::RRT<NodeType_> RRT = gidisizi::RRT<NodeType_>();
  int nodeId = 0 ;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::VectorXd::Ones(2) * 0);
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::VectorXd::Ones(2) * 0.5);
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lower = -Eigen::VectorXd::Ones(2);
  Eigen::VectorXd accuracy = 0.05*Eigen::VectorXd::Ones(2);
  RRT.initilize(q_init,q_goal,accuracy,upper,lower,10000,5.0);
  RRT.Plan();
  RRT.drawGraph();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testNode();
  testRRT();
  return 0;
}

