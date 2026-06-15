#pragma once

struct Detection
{
    bool valid = false;

    double u = 0.0;  // image x or bearing
    double v = 0.0;  // image y or elevation
    double z = 0.0;  // depth / altitude (3-D extension)

    double timestamp = 0.0;
};
