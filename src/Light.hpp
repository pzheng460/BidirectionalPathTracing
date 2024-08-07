#ifndef RAYTRACING_LIGHT_H
#define RAYTRACING_LIGHT_H

#pragma once

#include "Vector.hpp"

class Light
{
public:
    virtual ~Light() = default;
    Vector3f position;
    Vector3f intensity;
};

#endif //RAYTRACING_LIGHT_H