#include "Camera.hpp"

Ray Camera::generateRay(int i, int j) {
    float x = (2 * (j + 0.5) / (float)width - 1) * imageAspectRatio * scale; // calculate the x value of the pixel
    float y = (1 - 2 * (i + 0.5) / (float)height) * scale;
    Vector3f dir = normalize(Vector3f(-x, y, 1));
    return Ray(position, dir);
}

Ray Camera::sample(float* pdf, float* We, int* coorX, int* coorY) {
    *coorX = rand() % width;
    *coorY = rand() % height;
    float x = (2 * (*coorX + 0.5) / (float)width - 1) * imageAspectRatio * scale; // calculate the x value of the pixel
    float y = (1 - 2 * (*coorY + 0.5) / (float)height) * scale;
    float dist = Vector3f(-x, y, 1).norm();
    Vector3f dir = normalize(Vector3f(-x, y, 1));
    float A = 4 * scale * scale;
    *We = 1.0f / (A * cos_theta * cos_theta * cos_theta * cos_theta);
    *pdf = dist * dist / cos_theta;
    return Ray(position + Vector3f(-x, y, 1), dir);
}