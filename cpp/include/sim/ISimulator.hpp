#pragma once

#include "models/State.hpp"
#include "models/TargetState.hpp"
#include "models/ControlCommand.hpp"
#include "models/SensorData.hpp"

class ISimulator
{
public:
    virtual ~ISimulator() = default;

    virtual void update(const ControlCommand& cmd, double dt) = 0;

    virtual State       getDroneState()  const = 0;
    virtual TargetState getTargetTruth() const = 0;
    virtual SensorData  getSensors()     const = 0;
};
