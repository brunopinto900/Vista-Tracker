#pragma once

#include "mapping/IESDFMap.hpp"
#include "world/World.hpp"

class FakeESDFMap : public IESDFMap
{
public:
    explicit FakeESDFMap(const World& world);

    // Exact 2-D box SDF; z is ignored (obstacles are flat).
    float query(double x, double y, double z) const override;

private:
    const World& world_;
};
