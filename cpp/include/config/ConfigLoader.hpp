#pragma once

#include "config/Config.hpp"
#include <string>

namespace YAML { class Node; }

class ConfigLoader
{
public:
    static Config load(const std::string& path);

private:
    static void applyNode(Config& cfg, const YAML::Node& node);
};
