#include "ros/ros.h"
#include <iostream>
#include "gidisizi/utils/Node.hpp"
#include "gidisizi/RRT/RRTStar.hpp"
#include "gidisizi/Environment2D.hpp"
#include "gidisizi/utils/visualization.hpp"

int counter = 0;
using NodeType_ = gidisizi::Node<2>;
using Environment_ = gidisizi::Environment2d;

void testRRTStar(){

  Environment_* env = new Environment_();
  env->addWall(Eigen::Vector2d(-1.1,0.5),Eigen::Vector2d(0.5,0.6));
  env->addWall(Eigen::Vector2d(-0.5,-0.4),Eigen::Vector2d(1.1,-0.2));

  gidisizi::RRTStar<NodeType_,Environment_> RRTStar_ = gidisizi::RRTStar<NodeType_,Environment_>();
  int nodeId = 0;
  NodeType_* q_init = new NodeType_(nodeId++, Eigen::Vector2d(-0.7,0.9));
  NodeType_* q_goal = new NodeType_(nodeId++, Eigen::Vector2d(0.8,-0.8));
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd lower = -Eigen::VectorXd::Ones(2);
  Eigen::VectorXd accuracy = 0.1*Eigen::VectorXd::Ones(2);


  //std::cerr<<"1"<<std::endl;
  RRTStar_.initilize(env,q_init,q_goal,accuracy,upper,lower,10000,0.6);

  std::cerr << "RRT* starts!!" << std::endl;
  RRTStar_.Plan();

  //std::cerr<<"3"<<std::endl;
  //RRTStar_.drawGraph();
  //RRTStar_.drawGraphEvolution();
  path_vis::drawGraphEvolution(RRTStar_,env);

  std::cerr << "RRT* ends!!" << std::endl;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testRRTStar();
  return 0;
}
