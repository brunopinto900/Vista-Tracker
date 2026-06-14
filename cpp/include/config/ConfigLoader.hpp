#pragma once

#include <string>

#include "Config.hpp"

class ConfigLoader
{
public:

    static Config load(
        const std::string& filename);
};