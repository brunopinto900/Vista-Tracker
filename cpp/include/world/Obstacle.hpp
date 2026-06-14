#pragma once

struct Obstacle
{
    double x, y;
    double size;

    bool contains(double px, double py) const
    {
        return (px > x - size &&
                px < x + size &&
                py > y - size &&
                py < y + size);
    }
};
