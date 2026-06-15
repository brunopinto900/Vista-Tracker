#pragma once

#include "perception/IPerception.hpp"
#include "sim/ISimulator.hpp"

class GroundTruthPerception : public IPerception
{
public:
    explicit GroundTruthPerception(const ISimulator& sim);

    Detection update() override;

private:
    const ISimulator& sim_;
};
