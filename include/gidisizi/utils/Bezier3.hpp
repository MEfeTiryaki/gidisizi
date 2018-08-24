#pragma once

#include "gidisizi/utils/Curve.hpp"

namespace gidisizi {
class Bezier3 :public Curve
{
 public:
  // Constructors
  Bezier3(Eigen::VectorXd qStart,Eigen::VectorXd qEnd,Eigen::VectorXd qdStart):
    Curve(qStart,qEnd),
    qdStart_(qdStart)
  {
    P_ = Eigen::MatrixXd::Zero(2,3);
    P_.col(0) = qStart;
    P_.col(2) = qEnd;

    C_ = Eigen::MatrixXd::Zero(3,3);
    Cd_ = Eigen::MatrixXd::Zero(3,2);
    Cdd_ = Eigen::MatrixXd::Zero(3,1);
    C_ <<  1 ,-2 , 1
        , -2 , 2 , 0
        ,  1 , 0 , 0;
    Cd_ <<  2 ,-2
         , -4 , 2
         ,  2 , 0 ;
    Cdd_ <<  2
          , -4
          ,  2 ;
  };

  // Destructor.
  ~Bezier3(){};

  void setControlPointParameter(double t){
    P_.col(1) = P_.col(0)+qdStart_ * t ;
  }
  virtual Eigen::VectorXd getValue(double s) override{
    return P_*C_* Eigen::Vector3d(s*s,s,1);
  }
  Eigen::VectorXd getFirstDerivative(double s){
    return P_*Cd_* Eigen::Vector2d(s,1);
  }
  Eigen::VectorXd getSecondDerivative(double s){
    return P_*Cd_;
  }

 protected:
  Eigen::MatrixXd P_;
  Eigen::VectorXd qdStart_;

  Eigen::MatrixXd C_;
  Eigen::MatrixXd Cd_;
  Eigen::MatrixXd Cdd_;
};
}  // gidisizi
