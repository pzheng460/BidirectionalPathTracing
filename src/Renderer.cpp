#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <mutex>
#include <vector>
#include <thread>

std::atomic<int> g_complateTotals = 0;
std::mutex g_mutex;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.

void render_thread(std::vector<Vector3f>& framebuffer0, std::vector<Vector3f>& framebuffer1, const Scene& scene, int spp, int y0, int y1)
{
    for (int i = y0; i < y1; i++) {
        for (int j = 0; j < scene.width; j++) {
            int index = i * scene.width + j;
            for (int k = 0; k < spp; k++) {
                framebuffer0[index] += scene.pathTracing(scene.camera_->generateRay(i, j), 0) / spp;
//                framebuffer0[index] += scene.bidirectionalPathTracing(scene.camera_->generateRay(i, j), framebuffer1) / spp;
            }
        }
        {
            std::lock_guard<std::mutex> guard(g_mutex);
            UpdateProgress(++g_complateTotals / (float)scene.height);
        }
    }
}

void Renderer::Render(const Scene& scene, int spp)
{
    std::vector<Vector3f> framebuffer0(scene.width * scene.height); // camera map
    std::vector<Vector3f> framebuffer1(scene.width * scene.height); // light map

    g_complateTotals = 0;

    // change the spp value to change sample amount
    std::cout << "Samples Per Pixel: " << spp << "\n";

    int numThreads = std::thread::hardware_concurrency();  // get the number of threads supported by the hardware
//    numThreads = 1;
    int lines = scene.height / numThreads + 1;  // divide the height of the image by the number of threads 按照高度切割屏幕再分割给每个线程
    std::vector<std::thread> workers;  // create a vector of threads

    for (int i = 0; i < numThreads; i++) {
        int y0 = i * lines;
        int y1 = std::min(scene.height, (i + 1) * lines);
        workers.emplace_back(render_thread, std::ref(framebuffer0), std::ref(framebuffer1), std::ref(scene), spp, y0, y1);
    }

    std::cout << "Number of Threads: " << numThreads << "\n" << std::endl;

    for (auto& t : workers) {
        t.join();
    }

    for (auto& pixel : framebuffer1) {
        pixel = pixel / spp;
    }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer0[i].x + framebuffer1[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer0[i].y + framebuffer1[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer0[i].z + framebuffer1[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
