#pragma once

#include "models/State.hpp"
#include "models/TargetState.hpp"
#include "models/ControlCommand.hpp"
#include "world/Obstacle.hpp"
#include <vector>

class ISimulator
{
public:
    virtual State       getDroneState()  const = 0;
    virtual TargetState getTargetState() const = 0;

    virtual void applyControl(const ControlCommand& u, double dt) = 0;
    virtual void stepTarget(double dt) = 0;

    virtual std::vector<Obstacle> getVisibleObstacles() const = 0;

    virtual ~ISimulator() = default;
};
