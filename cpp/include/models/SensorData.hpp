#pragma once

#include "models/RGBImage.hpp"
#include "models/DepthImage.hpp"

struct SensorData
{
    RGBImage  rgb;
    DepthImage depth;
};
