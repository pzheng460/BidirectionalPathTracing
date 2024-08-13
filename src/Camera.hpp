#ifndef RAYTRACING_CAMERA_HPP
#define RAYTRACING_CAMERA_HPP

#include "Vector.hpp"
#include "global.hpp"
#include "Ray.hpp"

class Camera {
public:
    Camera(Vector3f position, int width, int height, double fov) : position(position), width(width), height(height), fov(fov) {
        scale = tan(deg2rad(fov * 0.5));
        cos_theta = cos(deg2rad(fov * 0.5));
        imageAspectRatio = width / (float)height;
    }
    Vector3f position;
    Ray generateRay(int i, int j);
    Ray sample(Vector3f& lookAt, float* pdf, float* We, int* coorX, int* coorY);

private:
    int width, height;
    double fov;
    float scale, cos_theta, imageAspectRatio;
};

#endif //RAYTRACING_CAMERA_HPP
