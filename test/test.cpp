#include "ros/ros.h"
#include <iostream>
#include "gidisizi/utils/Node.hpp"
#include "gidisizi/Environment2D.hpp"
#include "gidisizi/utils/visualization.hpp"
#include <openrave-core.h>

int counter = 0;
using NodeType_ = gidisizi::Node<2>;
using Environment_ = gidisizi::Environment2d;
void testNode(){
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

void testEnvironment(){
  Environment_* env = new Environment_();
  env->addWall(Eigen::Vector2d(-1.0,-1.0),Eigen::Vector2d(1.0,1.0));

  // no collision
  if(env->checkCollisions(Eigen::Vector2d(2.0,2.0),Eigen::Vector2d(3.0,3.0) )){
    std::cerr<< "1"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(0.0,2.0),Eigen::Vector2d(0.0,3.0) )){
    std::cerr<< "2"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(-2.0,2.0),Eigen::Vector2d(-3.0,3.0) )){
    std::cerr<< "3"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(-2.0,0.0),Eigen::Vector2d(-3.0,0.0) )){
    std::cerr<< "4"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(-2.0,-2.0),Eigen::Vector2d(-3.0,-3.0) )){
    std::cerr<< "5"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(0.0,-2.0),Eigen::Vector2d(0.0,-3.0) )){
    std::cerr<< "6"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(2.0,-2.0),Eigen::Vector2d(3.0,-3.0) )){
    std::cerr<< "7"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(2.0,0.0),Eigen::Vector2d(3.0,0.0) )){
    std::cerr<< "8"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(2.0,1.0),Eigen::Vector2d(1.0,2.0) )){
    std::cerr<< "9"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(-1.0,2.0),Eigen::Vector2d(-2.0,1.0) )){
    std::cerr<< "10"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(-2.0,-1.0),Eigen::Vector2d(-1.0,-2.0) )){
    std::cerr<< "11"<< std::endl;
  }
  if(env->checkCollisions(Eigen::Vector2d(1.0,-2.0),Eigen::Vector2d(2.0,-1.0) )){
    std::cerr<< "12"<< std::endl;
  }

  // collision
  if(!env->checkCollisions(Eigen::Vector2d(2.0,0.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "13"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(2.0,2.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "14"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(0.0,2.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "15"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(-2.0,2.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "16"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(-2.0,-2.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "17"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(0.0,-2.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "18"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(2.0,-2.0),Eigen::Vector2d(0.0,0.0))){
    std::cerr<< "19"<< std::endl;
  }

  if(!env->checkCollisions(Eigen::Vector2d(1.5, 0.0),Eigen::Vector2d(0.0,1.5))){
    std::cerr<< "20"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(0.0, 1.5),Eigen::Vector2d(-1.5,0.0))){
    std::cerr<< "21"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(-1.5, 0.0),Eigen::Vector2d(0.0,-1.5))){
    std::cerr<< "21"<< std::endl;
  }
  if(!env->checkCollisions(Eigen::Vector2d(0.0, -1.5),Eigen::Vector2d(1.5,0.0))){
    std::cerr<< "22"<< std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testNode();
  testEnvironment();
  return 0;
}
