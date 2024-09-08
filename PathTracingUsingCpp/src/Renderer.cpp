//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include <mutex>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.


void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 9;
    std::cout << "SPP: " << spp << "\n";

    int threadCount = 32;
    int threadStep = scene.height / threadCount;
    std::vector<std::thread> rays(threadCount);

    int prog = 0;
    std::mutex lock;
    auto tracing = [&] (int lrow, int hrow) {
        for (uint32_t j = lrow; j < hrow; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                // float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                //         imageAspectRatio * scale;
                // float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
                float step = 1.0f / scene.width;

                for (int k = 0; k < spp; k++){

                    float x = (2 * (i + step / 2 + step * (k % scene.width)) / (float)scene.width - 1) * imageAspectRatio * scale;
                    float y = (1 - 2 * (j + step / 2 + step * (k / scene.height)) / (float)scene.height) * scale;
                    
                    Vector3f dir = normalize(Vector3f(-x, y, 1));

                    // framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir + Vector3f(get_random_float() / 10000, get_random_float() / 10000, get_random_float() / 10000)), 0) / spp;
                    framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }
            lock.lock();
            UpdateProgress((++ prog) / (float)scene.height);
            lock.unlock();
        }
    };

    #pragma omp parallel for
    for (int i = 0; i < threadCount; i ++ ) {
        rays[i] = (std::thread(tracing, i * threadStep, (i + 1) * threadStep));
    }

    for (int i = 0; i < threadCount; i ++ ) {
        rays[i].join();
    }

    UpdateProgress(1.f);

    // auto index = [&](int i, int j) {
    //     return j * scene.width + i;
    // };

    // for (uint32_t j = 1; j < scene.height; ++j) {
    //     for (uint32_t i = 1; i < scene.width; ++i) {
    //         auto var = 
    //             framebuffer[index(i, j)] +
    //             framebuffer[index(i - 1, j)] +
    //             framebuffer[index(i, j - 1)] +
    //             framebuffer[index(i - 1, j - 1)];
    //         framebuffer[index(i, j)] = var / 4;
    //     }
    // }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
