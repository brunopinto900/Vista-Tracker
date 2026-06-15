#pragma once

#include "perception/IPerception.hpp"
#include "sim/ISimulator.hpp"

class GroundTruthPerception : public IPerception
{
public:
    explicit GroundTruthPerception(const ISimulator& sim)
        : sim_(sim) {}

    Detection update() override
    {
        TargetState tr = sim_.getTargetTruth();
        return { true, tr.x, tr.y, tr.z, 0.0 };
    }

private:
    const ISimulator& sim_;
};
