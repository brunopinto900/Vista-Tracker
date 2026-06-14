#pragma once

#include "models/State.hpp"
#include "models/TargetState.hpp"
#include "models/ControlCommand.hpp"

class IController
{
public:

    virtual ~IController() = default;

    virtual ControlCommand update(
        const State& drone,
        const TargetState& target,
        double dt) = 0;
};