#include "ros/ros.h"
#include <iostream>
#include "gidisizi/utils/Node.hpp"
#include "gidisizi/RRT/RRTX.hpp"
#include "gidisizi/Environment2D.hpp"
#include "gidisizi/utils/visualization.hpp"

int counter = 0;
using NodeType_ = gidisizi::Node<2>;
using Environment_ = gidisizi::Environment2d;

void testRRTX(){

  Environment_* env = new Environment_();
  env->addWall(Eigen::Vector2d(-1.1,0.5),Eigen::Vector2d(0.5,0.6));
  env->addWall(Eigen::Vector2d(-0.5,-0.4),Eigen::Vector2d(1.1,-0.2));

  gidisizi::RRTX<NodeType_,Environment_> RRTX_ = gidisizi::RRTX<NodeType_,Environment_>();
  int nodeId = 0;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::Vector2d(-0.7,0.8));
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::Vector2d(0.8,-0.8));
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lower = -Eigen::VectorXd::Ones(2);
  Eigen::VectorXd accuracy = 0.1*Eigen::VectorXd::Ones(2);


  //std::cerr<<"1"<<std::endl;
  RRTX_.initilize(env,q_init,q_goal,accuracy,upper,lower,10000,1.0);

  std::cerr << "RRTx starts!!" << std::endl;
  RRTX_.Plan();

  //std::cerr<<"3"<<std::endl;
  //RRTStar_.drawGraph();
  //RRTX_.drawGraphEvolution();
  path_vis::drawGraphEvolution(RRTX_,env);

  std::cerr << "RRTx ends!!" << std::endl;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testRRTX();
  return 0;
}
