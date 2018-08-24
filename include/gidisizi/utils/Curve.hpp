#pragma once

#include "gidisizi/utils/Node.hpp"
#include <vector>
#include "Eigen/Dense"
#include <iostream>

namespace gidisizi {
class Curve
{
 public:
  // Constructors
  Curve(Eigen::VectorXd qStart,Eigen::VectorXd qEnd):
    qStart_(qStart),
    qEnd_(qEnd)
  {
  };
  // Destructor.
  ~Curve(){};

  virtual Eigen::VectorXd getValue(double s){
    return Eigen::VectorXd();
  }

  virtual std::vector<Eigen::VectorXd> getDrawable(int division){
    double s = 0.0;
    double delta = 1.0/division ;
    std::vector<Eigen::VectorXd> result;
    for(int i = 0; i<=division; i++){
      s = delta*i ;
      result.push_back( getValue(s));
    }
    return result;
  }

 protected:
  Eigen::VectorXd qStart_;
  Eigen::VectorXd qEnd_;

};
}  // gidisizi
