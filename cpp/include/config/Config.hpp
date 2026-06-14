#pragma once

#include "../models/State.hpp"
#include "../models/TargetState.hpp"

struct SimulationConfig
{
    double dt{};
    double sim_time{};
};

struct PIDConfig
{
    double kp{};
    double ki{};
    double kd{};
};

struct ControllerConfig
{
    double desired_distance{};
    PIDConfig pid;
};

struct Config
{
    SimulationConfig simulation;

    State drone;

    TargetState target;

    ControllerConfig controller;
};