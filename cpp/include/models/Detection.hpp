#pragma once

struct Detection
{
    bool valid = false;

    double u  = 0.0;  // image x or bearing
    double v  = 0.0;  // image y or elevation
    double z  = 0.0;  // depth / altitude (3-D extension)

    double vx = 0.0;  // velocity — filled when the sensor provides it directly
    double vy = 0.0;  //   (e.g. ground-truth or Doppler); leave 0 for pixel-only detectors
    double vz = 0.0;
    bool has_velocity = false;  // estimator falls back to finite-diff when false

    double timestamp = 0.0;
};
