#pragma once

#include <vector>
#include <cstdint>

struct RGBImage
{
    int width  = 0;
    int height = 0;
    std::vector<uint8_t> data;  // row-major, 3 bytes per pixel (R,G,B)
};
