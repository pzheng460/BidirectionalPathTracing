#include "Camera.hpp"

Ray Camera::generateRay(int i, int j) {
    float x = (2 * (j + 0.5) / (float)width - 1) * imageAspectRatio * scale; // calculate the x value of the pixel
    float y = (1 - 2 * (i + 0.5) / (float)height) * scale;
    Vector3f dir = normalize(Vector3f(-x, y, 1));
    return Ray(position, dir);
}

Ray Camera::sample(Vector3f& lookAt, float* pdf, float* We, int* coorX, int* coorY) {
    Vector3f wi = lookAt - position;
    float dist = wi.norm();

    float x = -wi.x / wi.z;
    float y = wi.y / wi.z;
    *coorX = (x / (imageAspectRatio * scale) + 1) * 0.5 * width;
    *coorY = (1 - y / scale) * 0.5 * height;

    Vector3f dir = normalize(wi);
    float A = 4 * scale * scale;
    *We = 1.0f / (A * pow(cos_theta, 4));
    *pdf = dist * dist / cos_theta;
    return Ray(position, dir);
}