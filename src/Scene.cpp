#include "Scene.hpp"
#include <thread>

std::mutex framebuffer_mutex;

const float eps = 5e-4; // a small number to prevent self-intersection

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
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
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

Vector3f Scene::shade(const Intersection& hitObj, const Vector3f& wo, int depth) const
{
    // hitObj.m->hasEmission() is true if the object is a light source
    if (hitObj.m->hasEmission())
        return hitObj.m->getEmission(); // if the object is a light source, we don't need to consider the path tracing

    // Direct Lighting
    Vector3f L_dir;
    {
        float pdf_light;
        Intersection light;
        sampleLight(light, pdf_light); // uniformly sample a light source

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
                    L_indir = shade(inter, -wi, depth + 1) * f_r * cos_theta / pdf_hemi / RussianRoulette;
                }
            }
        }
    }

    return L_dir + L_indir;
}

// Implementation of Path Tracing
Vector3f Scene::pathTracing(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    auto hitObj = intersect(ray);
    if (!hitObj.happened)
        return Vector3f(0);
    return shade(hitObj, -ray.direction, depth);
}

Vector3f Scene::connectPath(std::vector<Vector3f>& framebuffer1, std::vector<PathVertex> &lightPath, std::vector<PathVertex> &cameraPath, int s, int t) const
{
    Vector3f L(0);

    int newIdx = 0;

    if (s == 0) // if the camera path hits a light source
    {
        if (t > 1 && cameraPath[t - 1].inter.m->hasEmission())
            L = cameraPath[t - 1].alpha * cameraPath[t - 1].inter.m->getEmission();
    }
    else
    {
        PathVertex light = lightPath[s - 1];
        PathVertex camera = cameraPath[t - 1];
        Vector3f f_s_light;
        Vector3f f_s_camera;

        if (s == 1)
        {
            // sample a point on the light source
            Intersection light_;
            float pdf_light; // P_A
            sampleLight(light_, pdf_light);

            light.inter = light_;
            light.alpha = light_.emit / pdf_light; // L_e / P_A

            f_s_light = Vector3f(1.0f);
        }

        if (t == 1)
        {
            float pdf = 1.0f;
            float We = 0.0;
            int coorX = 0, coorY = 0;
            Ray ray = camera_->sample(light.inter.coords, &pdf, &We, &coorX, &coorY);

            camera.inter.coords = ray.origin;
            camera.inter.normal = ray.direction;
            camera.pdfFwd = pdf;
            camera.alpha = We;

            f_s_camera = Vector3f(1.0f);
            if (coorX >= 0 && coorX < width && coorY >= 0 && coorY < height)
            {
                newIdx = coorY * width + coorX;
            }
            else
            {
                newIdx = -1;
            }
        }

        Vector3f cameraToLight = light.inter.coords - camera.inter.coords;
        Vector3f cameraToLightNormalized = cameraToLight.normalized();

        if (s > 1)
        {
            PathVertex lightPrev = lightPath[s - 2];
            f_s_light = light.inter.m->eval(-cameraToLightNormalized, (lightPrev.inter.coords - light.inter.coords).normalized(), light.inter.normal);
        }

        if (t > 1)
        {
            PathVertex cameraPrev = cameraPath[t - 2];
            f_s_camera = camera.inter.m->eval((cameraPrev.inter.coords - camera.inter.coords).normalized(), cameraToLightNormalized, camera.inter.normal);
        }

        auto inter = intersect(Ray(camera.inter.coords, cameraToLightNormalized));

        float cos_theta_light = std::max(0.0f, dotProduct(-cameraToLightNormalized, light.inter.normal));
        float cos_theta_camera = std::max(0.0f, dotProduct(cameraToLightNormalized, camera.inter.normal));

        if (inter.distance - cameraToLight.norm() > -eps) {
            auto g = cos_theta_light * cos_theta_camera / dotProduct(cameraToLight, cameraToLight);
            auto c = f_s_light * g * f_s_camera;
            L = light.alpha * camera.alpha * c;
        }
    }

    float misWeight = MISWeight(lightPath, cameraPath, s, t);
    L = L * misWeight;

    if (t == 1)
    {
        if (newIdx != -1) {
            std::lock_guard<std::mutex> lock(framebuffer_mutex);
            framebuffer1[newIdx] += L;
        }
        return {0};
    }

    return L;
}

Vector3f Scene::bidirectionalPathTracing(const Ray &ray, std::vector<Vector3f>& framebuffer1) const
{
    std::vector<PathVertex> lightPath, cameraPath;
    generateLightPath(lightPath);
    generateCameraPath(cameraPath, ray);
    Vector3f L(0);

    for (int s = 0; s <= lightPath.size(); s++) {
        for (int t = 1; t <= cameraPath.size(); t++) {
            L += connectPath(framebuffer1, lightPath, cameraPath, s, t);
        }
    }

    return L;
}

void Scene::generateLightPath(std::vector<PathVertex>& lightPath) const
{
    // y0
    Intersection light;
    float pdf_light; // P_A
    sampleLight(light, pdf_light);

    PathVertex v0;
    v0.inter = light;
    v0.alpha = light.emit / pdf_light; // L_e / P_A
    Vector3f wo_0 = light.m->sample(Vector3f(0, 0, 0), light.normal).normalized();
    v0.pdfFwd = light.m->pdf(Vector3f(0, 0, 0), wo_0, light.normal); // directional pdf
    lightPath.push_back(v0);

    int i = 1;
    Vector3f f_s(1.0);
    PathVertex pre = v0;
    Ray currentRay(light.coords, wo_0);  // Assuming light emits along normal direction
    while (true) {
        if (pre.pdfFwd < eps && i != 1) break;

        Intersection inter = intersect(currentRay);

        if (!inter.happened) break;
//        if (inter.m->hasEmission()) break;
        if (i >= maxDepth + 1) break;

        PathVertex v;
        v.inter = inter;
        Vector3f wi = -currentRay.direction;
        float cos_theta = std::max(0.0f, dotProduct(-wi, pre.inter.normal));
        v.alpha = lightPath[i - 1].alpha * f_s * cos_theta / pre.pdfFwd;
        v.pdfRev = pre.pdfFwd;
        Vector3f wo = inter.m->sample(-currentRay.direction, inter.normal).normalized();
        f_s = inter.m->eval(wi, wo, inter.normal);
        v.pdfFwd = inter.m->pdf(wi, wo, inter.normal);
        lightPath.push_back(v);

        currentRay = Ray(inter.coords, wo);
        pre = v;

        i++;
    }
}

void Scene::generateCameraPath(std::vector<PathVertex> &cameraPath, const Ray& ray) const
{
    Ray currentRay = ray;

    PathVertex v0;
    v0.inter.coords = ray.origin;
    v0.inter.normal = ray.direction;
    v0.inter.m = new Material(DIFFUSE, Vector3f(0));
    v0.alpha = Vector3f(1);
    v0.pdfFwd = 1.0f;
    cameraPath.push_back(v0);

    int i = 1;
    Vector3f f_s(1.0f);
    PathVertex pre = v0;
    while (true) {
        if (pre.pdfFwd < eps) break;

        Intersection inter = intersect(currentRay);

        if (!inter.happened) break;
        if (i >= maxDepth + 1) break;

        PathVertex v;
        v.inter = inter;
        Vector3f wi = -currentRay.direction;
        float cos_theta = std::max(0.0f, dotProduct(-wi, v0.inter.normal));
        v.alpha = cameraPath[i - 1].alpha * f_s * cos_theta / pre.pdfFwd;
        v.pdfRev = pre.pdfFwd;
        Vector3f wo = inter.m->sample(-currentRay.direction, inter.normal).normalized();
        f_s = inter.m->eval(wi, wo, inter.normal);
        v.pdfFwd = inter.m->pdf(wi, wo, inter.normal);
        cameraPath.push_back(v);

//        if (inter.m->hasEmission()) break;

        currentRay = Ray(inter.coords, wo);
        pre = v;

        i++;
    }
}

float Scene::MISWeight(std::vector<PathVertex>& lightPath, std::vector<PathVertex>& cameraPath, int s, int t) const
{
    if (s + t == 2) return 1;

    float sumRi = 0;

    auto remap0 = [](float x) { return x != 0 ? x : 1; };

    PathVertex* qs = s > 0 ? &lightPath[s - 1] : nullptr;
    PathVertex* pt = t > 0 ? &cameraPath[t - 1] : nullptr;
    PathVertex* qsMinus = s > 1 ? &lightPath[s - 2] : nullptr;
    PathVertex* ptMinus = t > 1 ? &cameraPath[t - 2] : nullptr;

    // Update reverse densities
    if (pt) {
        pt->pdfRev = s > 0 ? qs->inter.m->pdf(qs->inter.normal, pt->inter.coords - qs->inter.coords, qs->inter.normal)
                           : pt->inter.m->pdf(pt->inter.normal, cameraPath[t - 2].inter.coords - pt->inter.coords, pt->inter.normal);
    }
    if (ptMinus) {
        ptMinus->pdfRev = s > 0 ? pt->inter.m->pdf(pt->inter.normal, qs->inter.coords - pt->inter.coords, pt->inter.normal)
                                : pt->inter.m->pdf(pt->inter.normal, cameraPath[t - 2].inter.coords - ptMinus->inter.coords, ptMinus->inter.normal);
    }
    if (qs) {
        qs->pdfRev = pt->inter.m->pdf(pt->inter.normal, qs->inter.coords - pt->inter.coords, pt->inter.normal);
    }
    if (qsMinus) {
        qsMinus->pdfRev = qs->inter.m->pdf(qs->inter.normal, qsMinus->inter.coords - qs->inter.coords, qs->inter.normal);
    }

    float ri = 1;
    for (int i = t - 1; i > 0; --i) {
        ri *= remap0(cameraPath[i].pdfRev) / remap0(cameraPath[i].pdfFwd);
        sumRi += ri;
    }

    ri = 1;
    for (int i = s - 1; i >= 0; --i) {
        ri *= remap0(lightPath[i].pdfRev) / remap0(lightPath[i].pdfFwd);
        sumRi += ri;
    }

    return 1 / (1 + sumRi);
}