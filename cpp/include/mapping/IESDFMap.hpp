#pragma once

class IESDFMap
{
public:
    virtual ~IESDFMap() = default;

    // Returns signed distance to the nearest obstacle surface (metres).
    // Positive  → outside obstacle.
    // Zero      → on the surface.
    // Negative  → inside obstacle.
    virtual float query(double x, double y, double z) const = 0;
};
