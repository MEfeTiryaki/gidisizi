#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

#include "gidisizi/Node.hpp"

namespace gidisizi {
class RRT
{
 public:
  // Constructor.
  RRT();
  // Destructor.
  virtual ~RRT();
  // Init
  virtual void initilize(gidisizi::Node q_init, gidisizi::Node q_goal, int maxIteration,
                         double maxTime);

 protected:
  gidisizi::Node q_init_;
  gidisizi::Node q_goal_;

  int maxIteration_;
  double maxTime_;

};
}  // gidisizi
