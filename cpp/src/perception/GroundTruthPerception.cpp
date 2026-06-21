#include "perception/GroundTruthPerception.hpp"

GroundTruthPerception::GroundTruthPerception(const ISimulator& sim)
    : sim_(sim) {}

Detection GroundTruthPerception::update()
{
    TargetState tr = sim_.getTargetTruth();
    return { true, tr.x, tr.y, tr.z, tr.vx, tr.vy, tr.vz, true, 0.0 };
}
