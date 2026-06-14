#pragma once

#include "models/State.hpp"
#include "models/ControlCommand.hpp"

class IController
{
public:
    virtual ControlCommand update(
        const State& drone,
        const State& desired,
        double dt) = 0;

    virtual ~IController() = default;
};
