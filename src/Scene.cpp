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
    PathVertex light = s > 0 ? lightPath[s - 1] : PathVertex();
    PathVertex camera = cameraPath[t - 1];
    PathVertex lightSample;
    PathVertex cameraSample;

    if (s == 0) // if the camera path hits a light source
    {
        if (t > 1 && camera.inter.m->hasEmission())
            L = camera.alpha * camera.inter.m->getEmission();
    }
    else
    {
        Vector3f f_s_light;
        Vector3f f_s_camera;

        if (s == 1)
        {
            // sample a point on the light source
            Intersection light_;
            float pdf_light; // P_A
            sampleLight(light_, pdf_light);

            lightSample.inter = light_;
            lightSample.inter.m = new Material(DIFFUSE, Vector3f(0));
            lightSample.alpha = light_.emit / pdf_light; // L_e / P_A
            lightSample.point_pdf = pdf_light;
            Vector3f wo_0 = light_.m->sample(Vector3f(0, 0, 0), light_.normal).normalized();
            lightSample.dir_pdf = light_.m->pdf(Vector3f(0, 0, 0), wo_0, light_.normal); // directional pdf
            lightSample.p = lightSample.dir_pdf;

            light = lightSample;

            f_s_light = Vector3f(1.0f);
        }

        if (t == 1)
        {
            float pdf = 1.0f;
            float We = 0.0;
            int coorX = 0, coorY = 0;
            Ray ray = camera_->sample(light.inter.coords, &pdf, &We, &coorX, &coorY);

            cameraSample.inter.coords = ray.origin;
            cameraSample.inter.normal = ray.direction;
            cameraSample.inter.m = new Material(DIFFUSE, Vector3f(0));
            cameraSample.point_pdf = 1.0f;
            cameraSample.dir_pdf = pdf;
            cameraSample.p = pdf;
            cameraSample.alpha = We;

            camera = cameraSample;

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

    float misWeight = MISWeight(lightPath, cameraPath, s, t, lightSample, cameraSample);
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
    v0.point_pdf = pdf_light;
    v0.alpha = light.emit / pdf_light; // L_e / P_A
    Vector3f wo_0 = light.m->sample(Vector3f(0, 0, 0), light.normal).normalized();
    v0.dir_pdf = light.m->pdf(Vector3f(0, 0, 0), wo_0, light.normal); // directional pdf
    v0.p = v0.dir_pdf;
    lightPath.push_back(v0);

    int i = 1;
    Vector3f f_s(1.0);
    PathVertex& pre = v0;
    Ray currentRay(light.coords, wo_0);  // Assuming light emits along normal direction
    while (true) {
        if (pre.p < eps && i != 1) break;

        Intersection inter = intersect(currentRay);

        if (!inter.happened) break;
        if (inter.m->hasEmission()) break;
        if (i >= maxDepth + 1) break;

        PathVertex v;
        v.inter = inter;
        Vector3f wi = -currentRay.direction;
        float cos_theta = std::max(0.0f, dotProduct(-wi, pre.inter.normal));
//        v.pdfFwd = pre.p * cos_theta * std::max(0.0f, dotProduct(wi, inter.normal)) / (dotProduct(inter.coords - pre.inter.coords, inter.coords - pre.inter.coords));
//        v.pdfFwd = pre.p;
        v.alpha = lightPath[i - 1].alpha * f_s * cos_theta / pre.p;
        Vector3f wo = inter.m->sample(-currentRay.direction, inter.normal).normalized();
        f_s = inter.m->eval(wi, wo, inter.normal);
        v.p = inter.m->pdf(wi, wo, inter.normal);
        lightPath.push_back(v);

//        lightPath[i - 1].pdfRev = inter.m->pdf(wo, wi, inter.normal) * std::max(0.0f, dotProduct(-wi, pre.inter.normal)) * std::max(0.0f, dotProduct(wi, inter.normal)) / (dotProduct(inter.coords - pre.inter.coords, inter.coords - pre.inter.coords));
//        lightPath[i - 1].pdfRev = inter.m->pdf(wo, wi, inter.normal);

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
    v0.p = 1.0f;
    v0.dir_pdf = 1.0f;
    v0.point_pdf = 1.0f;
    cameraPath.push_back(v0);

    int i = 1;
    Vector3f f_s(1.0f);
    PathVertex& pre = v0;
    while (true) {
        if (pre.p < eps) break;

        Intersection inter = intersect(currentRay);

        if (!inter.happened) break;
        if (i >= maxDepth + 1) break;

        PathVertex v;
        v.inter = inter;
        Vector3f wi = -currentRay.direction;
        float cos_theta = std::max(0.0f, dotProduct(-wi, pre.inter.normal));
//        v.pdfFwd = pre.p * cos_theta * std::max(0.0f, dotProduct(wi, inter.normal)) / (dotProduct(inter.coords - pre.inter.coords, inter.coords - pre.inter.coords));
//        v.pdfFwd = pre.p;
        v.alpha = cameraPath[i - 1].alpha * f_s * cos_theta / pre.p;
        Vector3f wo = inter.m->sample(-currentRay.direction, inter.normal).normalized();
        f_s = inter.m->eval(wi, wo, inter.normal);
        v.p = inter.m->pdf(wi, wo, inter.normal);
        cameraPath.push_back(v);

//        cameraPath[i - 1].pdfRev = inter.m->pdf(wo, wi, inter.normal) * std::max(0.0f, dotProduct(-wi, pre.inter.normal)) * std::max(0.0f, dotProduct(wi, inter.normal)) / (dotProduct(inter.coords - pre.inter.coords, inter.coords - pre.inter.coords));
//        cameraPath[i - 1].pdfRev = inter.m->pdf(wo, wi, inter.normal);
        if (inter.m->hasEmission()) break;

        currentRay = Ray(inter.coords, wo);
        pre = v;

        i++;
    }
}

double Scene::MISWeight(std::vector<PathVertex>& lightPath, std::vector<PathVertex>& cameraPath, int s, int t, PathVertex& lightSample, PathVertex& cameraSample) const
{
    // Initialize weight inverse and ratio
    double w_inv = 0.0, ratio = 1.0;

    // Traverse the camera path
    PathVertex cur_v, prev_v, next_v;
    for (int i = t - 1; i > 0; --i) {
        cur_v = cameraPath[i];
        if (i == t - 1) {
            prev_v = (s == 1) ? lightSample : lightPath[s - 1];
        } else {
            prev_v = cameraPath[i + 1];
        }
        next_v = cameraPath[i - 1];

        double nom, denom;
        double p, g;

        Vector3f wi = cur_v.inter.coords - prev_v.inter.coords;
        double dist = wi.norm();

        wi = cur_v.inter.coords - prev_v.inter.coords;
        wi = wi.normalized();

        g = fabs(std::max(0.0f, dotProduct(-wi, cur_v.inter.normal)) * std::max(0.0f, dotProduct(wi, prev_v.inter.normal))) / (dist * dist);

        // Compute the probability density function (PDF)
        if (s == 0 && i == t - 1)
        {
            // The path vertex is directly connected to the light source
            if ((lightSample.inter.coords - cur_v.inter.coords).norm() < eps)
            {
                p = lightSample.point_pdf;
                g = 1.0;
            }
            else
            {
                return 0.0;
            }
        }
        else if (s == 1 && i == t - 1)
        {
            // The path vertex is on the light source
            p = lightSample.point_pdf;
        }
        else if (s == 1 && i == t - 2)
        {
            p = lightSample.dir_pdf;
        }
        else
        {
            p = prev_v.inter.m->pdf(Vector3f(0, 0, 0), wi, prev_v.inter.normal);
        }
        nom = p * g;

        // Compute the denominator
        Vector3f wo = next_v.inter.coords - cur_v.inter.coords;
        dist = wo.norm();
        wo = wo.normalized();

        g = fabs(std::max(0.0f, dotProduct(wo, cur_v.inter.normal)) * std::max(0.0f, dotProduct(-wo, next_v.inter.normal))) / (dist * dist);

        if (i == 1)
        {
            // The vertex is on the camera lens
            p = 1.0;
            g = 1.0;
        }
        else
        {
            p = next_v.inter.m->pdf(Vector3f(0, 0, 0), -wo, next_v.inter.normal);
        }
        denom = p * g;

        // Update ratio and weight inverse
        ratio *= nom / denom;

        w_inv += ratio * ratio;
    }

    // Traverse the light path
    ratio = 1.0;
    for (int i = s - 1; i > 0; --i) {
        cur_v = lightPath[i];
        if (i == s - 1) {
            prev_v = (t == 1) ? cameraSample : cameraPath[t - 1];
        } else {
            prev_v = lightPath[i + 1];
        }
        next_v = lightPath[i - 1];

        double nom, denom;
        double p, g;

        Vector3f wi = cur_v.inter.coords - prev_v.inter.coords;
        double dist = wi.norm();
        wi = wi.normalized();

        g = fabs(std::max(0.0f, dotProduct(-wi, cur_v.inter.normal)) * std::max(0.0f, dotProduct(wi, prev_v.inter.normal))) / (dist * dist);

        if (t <= 1 && i == s - 1)
        {
            p = cameraSample.dir_pdf;
        }
        else
        {
            p = prev_v.inter.m->pdf(Vector3f(0, 0, 0), wi, prev_v.inter.normal);
        }
        nom = p * g;

        // Compute the denominator
        Vector3f wo = next_v.inter.coords - cur_v.inter.coords;
        dist = wo.norm();
        wo = wo.normalized();

        g = fabs(std::max(0.0f, dotProduct(wo, cur_v.inter.normal)) * std::max(0.0f, dotProduct(-wo, next_v.inter.normal))) / (dist * dist);
        if (i == 1)
        {
            p = next_v.point_pdf;
        }
        else
        {
            p = next_v.inter.m->pdf(Vector3f(0, 0, 0), -wo, next_v.inter.normal);
        }

        denom = p * g;

        ratio *= nom / denom;

        w_inv += ratio * ratio;
    }

    w_inv += 1.0;

    double w = 1.0 / w_inv;

    if (w < 1.0 || (s <= 1 && t <= 1)) {
        return w;
    } else {
        return 1.0;
    }
}