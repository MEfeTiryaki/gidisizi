#include "gidisizi/RRT.hpp"

namespace gidisizi {

RRT::RRT()
{
}

RRT::~RRT()
{
}

void RRT::initilize(gidisizi::Node q_init, gidisizi::Node q_goal, int maxIteration, double maxTime)
{
  q_init_ = q_init;
  q_goal_ = q_goal;
  maxIteration_ = maxIteration;
  maxTime_ = maxTime;
}

} /* namespace iyi*/
