#pragma once

#include <vector>

struct DepthImage
{
    int width  = 0;
    int height = 0;
    std::vector<float> data;  // row-major, metres per pixel
};
