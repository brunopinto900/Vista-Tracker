#include "perception/GroundTruthPerception.hpp"

GroundTruthPerception::GroundTruthPerception(const ISimulator& sim)
    : sim_(sim) {}

Detection GroundTruthPerception::update()
{
    TargetState tr = sim_.getTargetTruth();
    return { true, tr.x, tr.y, tr.z, 0.0 };
}
