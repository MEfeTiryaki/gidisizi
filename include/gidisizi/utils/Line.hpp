#pragma once
#include "gidisizi/utils/Curve.hpp"

namespace gidisizi {
class Line: public Curve
{
 public:
  // Constructors
  Line(Eigen::VectorXd qStart,Eigen::VectorXd qEnd):
    Curve(qStart,qEnd)
  {
  };
  // Destructor.
  ~Line(){};

  virtual Eigen::VectorXd getValue(double s) override{
    return qStart_ + s*(qEnd_-qStart_);
  }

};
}  // gidisizi
