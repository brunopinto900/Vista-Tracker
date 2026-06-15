#pragma once

#include "sim/ISimulator.hpp"
#include "world/World.hpp"

class KinematicSim : public ISimulator
{
public:
    KinematicSim(const State& d, const TargetState& t, const World& w);

    void update(const ControlCommand& cmd, double dt) override;

    State       getDroneState()  const override;
    TargetState getTargetTruth() const override;
    SensorData  getSensors()     const override;

private:
    State       drone_;
    TargetState target_;
    World       world_;
};
