#include "ros/ros.h"
#include <iostream>
#include "gidisizi/utils/Node.hpp"
#include "gidisizi/RRT/RRT.hpp"
#include "gidisizi/Environment2D.hpp"
#include "gidisizi/utils/visualization.hpp"

int counter = 0;
using NodeType_ = gidisizi::Node<2>;
using Environment_ = gidisizi::Environment2d;

void testRRTEmpty(){

  Environment_* env = new Environment_();

  gidisizi::RRT<NodeType_,Environment_> RRT = gidisizi::RRT<NodeType_,Environment_>();
  int nodeId = 0;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::Vector2d(0.0,0.0));
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::Vector2d(0.8,-0.8));
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lower = -Eigen::VectorXd::Ones(2);
  Eigen::VectorXd accuracy = 0.05*Eigen::VectorXd::Ones(2);



  RRT.initilize(env,q_init,q_goal,accuracy,upper,lower,2000,5.0);
  std::cerr << "RRT starts!!" << std::endl;
  RRT.Plan();
  //RRT.drawGraph();
  //RRT.drawGraphEvolution();
  path_vis::drawGraphEvolution(RRT,env);
  std::cerr << "RRT ends!!" << std::endl;
}

void testRRT(){

  Environment_* env = new Environment_();
  env->addWall(Eigen::Vector2d(-1.0,0.5),Eigen::Vector2d(0.5,0.6));
  env->addWall(Eigen::Vector2d(-0.5,-0.4),Eigen::Vector2d(1.0,-0.2));

  gidisizi::RRT<NodeType_,Environment_> RRT = gidisizi::RRT<NodeType_,Environment_>();
  int nodeId = 0;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::Vector2d(-0.7,0.8));
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::Vector2d(0.8,-0.8));
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lower = -Eigen::VectorXd::Ones(2);
  Eigen::VectorXd accuracy = 0.05*Eigen::VectorXd::Ones(2);



  RRT.initilize(env,q_init,q_goal,accuracy,upper,lower,2000,1.0);
  std::cerr << "RRT starts!!" << std::endl;
  RRT.Plan();
  //RRT.drawGraph();
  //RRT.drawGraphEvolution();
  path_vis::drawGraphEvolution(RRT,env);
  std::cerr << "RRT ends!!" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testRRT();
  //testRRTEmpty();
  return 0;
}
