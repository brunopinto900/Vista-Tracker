#pragma once

#include "models/State.hpp"
#include "models/ControlCommand.hpp"

class Simulator
{
public:

    void step(
        State& drone,
        const ControlCommand& command,
        double dt);
};