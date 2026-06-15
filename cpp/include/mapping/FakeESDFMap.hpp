#pragma once

#include "mapping/IESDFMap.hpp"
#include "world/World.hpp"
#include <cmath>
#include <limits>

class FakeESDFMap : public IESDFMap
{
public:
    explicit FakeESDFMap(const World& world) : world_(world) {}

    // Exact 2-D box SDF; z is ignored (obstacles are flat).
    float query(double x, double y, double /*z*/) const override
    {
        float min_dist = std::numeric_limits<float>::max();

        for (const auto& o : world_.obstacles)
        {
            double dx = std::abs(x - o.x) - o.size;
            double dy = std::abs(y - o.y) - o.size;

            double outside = std::sqrt(
                std::max(dx, 0.0) * std::max(dx, 0.0) +
                std::max(dy, 0.0) * std::max(dy, 0.0));

            double inside = std::min(std::max(dx, dy), 0.0);

            float d = static_cast<float>(outside + inside);
            if (d < min_dist)
                min_dist = d;
        }

        return min_dist;
    }

private:
    const World& world_;
};
