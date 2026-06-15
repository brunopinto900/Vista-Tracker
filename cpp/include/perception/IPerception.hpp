#pragma once

#include "models/Detection.hpp"

class IPerception
{
public:
    virtual ~IPerception() = default;

    virtual Detection update() = 0;
};
