#pragma once

#include "gidisizi/utils/Node.hpp"
#include <vector>
#include "Eigen/Dense"

#include <iostream>
namespace gidisizi {
struct Box
{
  Eigen::Vector2d point1;
  Eigen::Vector2d point2;
};
struct VelocityEnvelop
{
  Eigen::Vector2d x;
  Eigen::Vector2d y;
};
class Environment4d
{
 public:
  // Constructors
  Environment4d()
      : walls_()
  {
  }
  ;

  // Destructor.
  virtual ~Environment4d()
  {
  }
  ;
  // Init
  virtual void initilize()
  {
  }
  ;
  virtual void addWall(Eigen::Vector2d edge1, Eigen::Vector2d edge2)
  {
    gidisizi::Box w;
    w.point1 = edge1;
    w.point2 = edge2;
    addWall(w);
  }
  ;
  virtual void addWall(gidisizi::Box w)
  {
    walls_.push_back(w);
  }
  ;
  virtual std::vector<gidisizi::Box> getWalls()
  {
    return walls_;
  }
  ;

  virtual void setVelocityEnvelop(Eigen::Vector2d x, Eigen::Vector2d y)
  {
    gidisizi::VelocityEnvelop v;
    v.x = x;
    v.x = y;
    setVelocityEnvelop(v);
  }
  ;
  virtual void setVelocityEnvelop(gidisizi::VelocityEnvelop v)
  {
    velocityLimits_ = v ;
  }
  ;
  virtual bool checkCollisions(Eigen::Vector4d start, Eigen::Vector4d end)
  {
    if(end[3]<velocityLimits_.x[0] || end[3]>velocityLimits_.x[1]
    || end[4]<velocityLimits_.y[0] || end[4]>velocityLimits_.y[1]){
      return true;
    }
    for (auto w :walls_) {
      if((start[0]>w.point2[0] && end[0]<w.point2[0])
      &&(end[1]<w.point2[1] && end[1]>w.point1[1])){
        return true;
      }
      if((start[0]<w.point1[0] && end[0]>w.point1[0])
      &&(end[1]<w.point2[1] && end[1]>w.point1[1])){
        return true;
      }
      if((start[1]>w.point2[1] && end[1]<w.point2[1])
      &&(end[0]<w.point2[0] && end[0]>w.point1[0])){
        return true;
      }
      if((start[1]<w.point1[1] && end[1]>w.point1[1])
      &&(end[0]<w.point2[0] && end[0]>w.point1[0])) {
        return true;
      }

      double y = (end[1] - start[1]) / (end[0] - start[0]) * (w.point1[0] - start[0]) + start[1];
      //std::cerr<<"y1 : "<<y<<std::endl;
      if (w.point1[1] < y && w.point2[1] > y) {
        return true;
      }
      y = (end[1] - start[1]) / (end[0] - start[0]) * (w.point2[0] - start[0]) + start[1];
      //std::cerr<<"y2 : "<<y<<std::endl;
      if (w.point1[1] < y && w.point2[1] > y) {
        return true;
      }
      double x = (end[0] - start[0]) / (end[1] - start[1]) * (w.point1[1] - start[1]) + start[0];
      //std::cerr<<"x1 : "<<x<<std::endl;
      if (w.point1[0] < x && w.point2[0] > x) {
        return true;
      }
      x = (end[0] - start[0]) / (end[1] - start[1]) * (w.point2[1] -start[1]) + start[0];
      //std::cerr<<"x2 : "<<x<<std::endl;
      if (w.point1[0] < x && w.point2[0] > x) {
        return true;
      }
    }
    return false;
  }

 protected:
  std::vector<gidisizi::Box> walls_;
  gidisizi::VelocityEnvelop velocityLimits_;
};
}  // gidisizi
