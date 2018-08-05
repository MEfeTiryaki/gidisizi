#include "ros/ros.h"
#include <iostream>
#include "gidisizi/utils/Node.hpp"
#include "gidisizi/examples/RRTNonHolonomic.hpp"
#include "gidisizi/Environment2D.hpp"
#include "gidisizi/utils/visualization.hpp"

int counter = 0;
using NodeType_ = gidisizi::Node<3>;
using Environment_ = gidisizi::Environment2d;

void testRRTNonHolonomic(){

  Environment_* env = new Environment_();
  env->addWall(Eigen::Vector2d(-1.1,0.5),Eigen::Vector2d(0.5,0.6));
  env->addWall(Eigen::Vector2d(-0.5,-0.4),Eigen::Vector2d(1.1,-0.2));

  gidisizi::RRTNonHolonomic<NodeType_,Environment_> RRTNonHolonomic_= gidisizi::RRTNonHolonomic<NodeType_,Environment_>();
  int nodeId = 0;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::Vector3d(-0.7,0.9,0.0));
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::Vector3d(0.8,-0.8,0.0));
  Eigen::VectorXd upper = Eigen::VectorXd(3);
  Eigen::VectorXd lower = Eigen::VectorXd(3);
  lower << -1.0,-1.0,-M_PI;
  upper << 1.0,1.0,M_PI;
  Eigen::VectorXd accuracy = 0.1*Eigen::VectorXd::Ones(3);


  //std::cerr<<"1"<<std::endl;
  RRTNonHolonomic_.initilize(env,q_init,q_goal,accuracy,upper,lower,10000,5.0);

  std::cerr << "StateSpacePlan starts!!" << std::endl;
  RRTNonHolonomic_.Plan();
  std::cerr << "StateSpacePlan ends!!" << std::endl;
  //std::cerr<<"3"<<std::endl;
  //StateSpacePlanStar_.drawGraph();
  //StateSpacePlanStar_.drawGraphEvolution();
  path_vis::drawGraphEvolution(RRTNonHolonomic_,env);



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testRRTNonHolonomic();
  return 0;
}
