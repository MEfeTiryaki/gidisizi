#include "ros/ros.h"
#include <iostream>
#include "gidisizi/Node.hpp"
#include "gidisizi/RRT.hpp"

int counter = 0;

void testNode()
{
  gidisizi::Node* node_1 = new gidisizi::Node();
  gidisizi::Node* node_2 = new gidisizi::Node(2);
  gidisizi::Node* node_3 = new gidisizi::Node(3, Eigen::VectorXd::Ones(5) * 5);
  gidisizi::Node* node_4 = new gidisizi::Node(4, Eigen::VectorXd::Ones(5) * 4, node_3);
  gidisizi::Node* node_5 = new gidisizi::Node(5, Eigen::VectorXd::Ones(5) * 2, node_3);
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
  if (node_3->getchildren().size() != 2) {
    std::cerr
        << "11 - Node(int,Eigen::VectorXd,gidisizi::Node) constructor: parent assignment not working properly"
        << std::endl;
  }
}

void testRRT(){
  gidisizi::RRT RRT = gidisizi::RRT();
  int nodeId = 0 ;
  gidisizi::Node q_init = gidisizi::Node(nodeId++, Eigen::VectorXd::Ones(5) * 0);
  gidisizi::Node q_goal = gidisizi::Node(nodeId++, Eigen::VectorXd::Ones(5) * 2);

  RRT.initilize(q_init,q_goal,100,1.0);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testNode();
  testRRT();
  return 0;
}

