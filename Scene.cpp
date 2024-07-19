//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f Scene::shade(const Intersection& hitObj, const Vector3f& wo) const
{
    // hitObj.m->hasEmission() is true if the object is a light source
    if (hitObj.m->hasEmission())
        return hitObj.m->getEmission(); // if the object is a light source, we don't need to consider the path tracing

    const float eps = 5e-4; // a small number to prevent self-intersection

    // Direct Lighting
    Vector3f L_dir;
    {
        float pdf_light;
        Intersection light;
        sampleLight(light, pdf_light);

        Vector3f objToLight = light.coords - hitObj.coords;
        Vector3f wi = objToLight.normalized();

        // Check if the point is in shadow
        auto inter = intersect(Ray(hitObj.coords, wi));
        if (inter.distance - objToLight.norm() > - eps) { // if the ray is not blocked in the middle
            Vector3f L_i = light.emit;
            Vector3f f_r = hitObj.m->eval(wi, wo, hitObj.normal);
            float cos_theta = std::max(0.0f, dotProduct(wi, hitObj.normal));
            float cos_theta_light = std::max(0.0f, dotProduct(-wi, light.normal));

            L_dir = L_i * f_r * cos_theta * cos_theta_light / (dotProduct(objToLight, objToLight)) / pdf_light;
        }
    }

    // Indirect Lighting
    Vector3f L_indir;
    {
        if (get_random_float() < RussianRoulette) {
            Vector3f wi = hitObj.m->sample(wo, hitObj.normal).normalized();
            float pdf_hemi = hitObj.m->pdf(wi, wo, hitObj.normal);

            if (pdf_hemi > eps) {
                auto inter = intersect(Ray(hitObj.coords, wi));
                if (inter.happened && !inter.m->hasEmission()) { // If ray r hit a non-emitting object
                    Vector3f f_r = hitObj.m->eval(wi, wo, hitObj.normal);
                    float cos_theta = std::max(0.0f, dotProduct(wi, hitObj.normal));
                    L_indir = shade(inter, -wi) * f_r * cos_theta / pdf_hemi / RussianRoulette;
                }
            }
        }
    }

    return L_dir + L_indir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    auto hitObj = intersect(ray);
    if (!hitObj.happened)
        return Vector3f(0);
    return shade(hitObj, -ray.direction);
}