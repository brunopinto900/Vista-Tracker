#pragma once

struct Waypoint
{
    double x    = 0.0;
    double y    = 0.0;
    double z    = 0.0;
    double speed = 1.0;  // desired approach speed (m/s)
    double hold  = 0.0;  // seconds to stay at this waypoint before moving on
};
