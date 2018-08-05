#include "ros/ros.h"
#include <iostream>
#include "gidisizi/utils/Node.hpp"
#include "gidisizi/examples/StateSpacePlan.hpp"
#include "gidisizi/Environment4D.hpp"
#include "gidisizi/utils/visualization.hpp"

int counter = 0;
using NodeType_ = gidisizi::Node<4>;
using Environment_ = gidisizi::Environment4d;

void testStateSpacePlan(){

  Environment_* env = new Environment_();
  env->addWall(Eigen::Vector2d(-1.1,0.5),Eigen::Vector2d(0.5,0.6));
  env->addWall(Eigen::Vector2d(-0.5,-0.4),Eigen::Vector2d(1.1,-0.2));
  env->setVelocityEnvelop(Eigen::Vector2d(-0.5,0.5),Eigen::Vector2d(-0.5,0.5));

  gidisizi::StateSpacePlan<NodeType_,Environment_> StateSpacePlan_= gidisizi::StateSpacePlan<NodeType_,Environment_>();
  int nodeId = 0;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::Vector4d(-0.7,0.9,0.0,0.0));
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::Vector4d(0.8,-0.8,0.0,0.0));
  Eigen::VectorXd upper = Eigen::VectorXd(4);
  Eigen::VectorXd lower = Eigen::VectorXd(4);
  lower << -1.0,-1.0,-0.5,-0.5;
  upper << 1.0,1.0,0.5,0.5;
  Eigen::VectorXd accuracy = 0.1*Eigen::VectorXd::Ones(4);


  //std::cerr<<"1"<<std::endl;
  StateSpacePlan_.initilize(env,q_init,q_goal,accuracy,upper,lower,10000,5.0);

  std::cerr << "StateSpacePlan starts!!" << std::endl;
  StateSpacePlan_.Plan();
  std::cerr << "StateSpacePlan ends!!" << std::endl;
  //std::cerr<<"3"<<std::endl;
  //StateSpacePlanStar_.drawGraph();
  //StateSpacePlanStar_.drawGraphEvolution();
  path_vis::drawGraphEvolution(StateSpacePlan_,env);



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testStateSpacePlan();
  return 0;
}
