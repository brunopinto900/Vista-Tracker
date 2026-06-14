#pragma once

#include "sim/ISimulator.hpp"
#include "world/World.hpp"
#include "sensing/CameraModel.hpp"

class KinematicSim : public ISimulator
{
private:
    State       drone;
    TargetState target;
    World       world;
    CameraModel camera;

public:
    KinematicSim(const State& d, const TargetState& t, const World& w)
        : drone(d), target(t), world(w) {}

    State       getDroneState()  const override { return drone;  }
    TargetState getTargetState() const override { return target; }

    void applyControl(const ControlCommand& u, double dt) override
    {
        drone.x += u.vx * dt;
        drone.y += u.vy * dt;
        drone.z += u.vz * dt;

        // keep velocity fields consistent so they can be logged
        drone.vx = u.vx;
        drone.vy = u.vy;
        drone.vz = u.vz;
    }

    void stepTarget(double dt) override
    {
        target.x += target.vx * dt;
        target.y += target.vy * dt;
        target.z += target.vz * dt;
    }

    std::vector<Obstacle> getVisibleObstacles() const override
    {
        std::vector<Obstacle> visible;

        for (const auto& o : world.obstacles)
            if (camera.isVisible(drone, o))
                visible.push_back(o);

        return visible;
    }
};
