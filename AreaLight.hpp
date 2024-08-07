#ifndef RAYTRACING_AREALIGHT_H
#define RAYTRACING_AREALIGHT_H

#pragma once

#include "Vector.hpp"
#include "Light.hpp"
#include "Triangle.hpp"
#include "global.hpp"

class AreaLight : public Light, public MeshTriangle
{
public:
    AreaLight(const std::string& filename, Material *mt = new Material()) : MeshTriangle(filename, mt)
    {
        Light::position = getPosition();
        Light::intensity = getIntensity();
        u = Vector3f(1, 0, 0);
        v = Vector3f(0, 0, 1);
    }

    Vector3f SamplePoint() const
    {
        auto random_u = get_random_float();
        auto random_v = get_random_float();
        return position + random_u * u + random_v * v;
    }

    Vector3f u;
    Vector3f v;
};

#endif //RAYTRACING_AREALIGHT_H