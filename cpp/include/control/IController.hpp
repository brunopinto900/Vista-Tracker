#pragma once

#include "models/State.hpp"
#include "models/Reference.hpp"
#include "models/ControlCommand.hpp"

class IController
{
public:
    virtual ~IController() = default;

    virtual ControlCommand update(
        const State& drone,
        const Reference& reference,
        double dt) = 0;
};
