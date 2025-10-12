#pragma once

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

class VADERCustomObjective : public ompl::base::MultiOptimizationObjective
{
public:
  VADERCustomObjective(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::MultiOptimizationObjective(si)
    {
      addObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si), 10.0);
      addObjective(std::make_shared<ompl::base::MaximizeMinClearanceObjective>(si), 1.0);
    }
};
