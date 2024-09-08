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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{

    
    // TO DO Implement Path Tracing Algorithm here

    // 光线和场景中所有的物体求交点
    Intersection inter = Scene::intersect(ray);

    // 没有交点
    if (!inter.happened)
        return Vector3f();

    // 交点在光源，直接返回光源的采样
    if (inter.m->hasEmission())
        return inter.m->getEmission();

    // 直接光照
    Vector3f L_dir;

    // 间接光照
    Vector3f L_indir;

    // 采样光源点
    Intersection lightInter;
    float lightPdf;
    sampleLight(lightInter, lightPdf);

    // 交点信息
    auto pos = inter.coords;
    auto view2Pos = ray.direction;

    // 采样光源点信息
    auto lightPos = lightInter.coords;

    // 预计算
    auto pos2Light = (lightPos - pos).normalized();
    auto dist = (lightPos - pos).norm();

    switch(inter.m->getType()) {
        case DIFFUSE:
        {
            // 射出一条从交点到采样光源点并计算长度
            // 如果与交点到采样光源点的距离相同说明没有障碍物遮挡可以计算实际的直接光照
            if (fabs(Scene::intersect(Ray(pos, pos2Light)).distance - dist) < 0.001) {
                auto L_i = lightInter.emit;
                auto f_r = inter.m->eval(view2Pos, pos2Light, inter.normal.normalized());
                float cosOnObj = dotProduct(inter.normal, pos2Light);
                float cosOnLight = dotProduct(lightInter.normal, -pos2Light);

                // 直接光照从光源面采样积分
                L_dir = L_i * f_r * cosOnObj * cosOnLight / (dist * dist) / lightPdf;
            }

            // 轮盘赌方式判断是否再次迭代
            if (get_random_float() < RussianRoulette) {
                // 发出一条 采样光 与场景求交
                auto sampleDir = inter.m->sample(view2Pos, inter.normal).normalized();
                Ray shot(pos, sampleDir);
                Intersection sampleInter = Scene::intersect(shot);

                // 如果采样光有交点 并且 不在光源（间接光照）
                if (sampleInter.happened && !sampleInter.m->hasEmission()) {
                    auto f_r = inter.m->eval(view2Pos, sampleDir, inter.normal.normalized());
                    float pdf = inter.m->pdf(view2Pos, sampleDir, inter.normal);
                    float cosOnObj = dotProduct(inter.normal, sampleDir);
                    if (pdf > EPSILON)
                        L_indir = castRay(shot, depth + 1) * f_r * cosOnObj / pdf / RussianRoulette;
                }
            }

            break;
        }
        case MIRROR:
        {
            // 轮盘赌方式判断是否再次迭代
            if (get_random_float() < RussianRoulette) {
                // 发出一条 采样光 与场景求交
                auto sampleDir = inter.m->sample(view2Pos, inter.normal).normalized();
                Ray shot(pos, sampleDir);
                Intersection sampleInter = Scene::intersect(shot);

                // 如果采样光有交点 并且 不在光源（间接光照）
                if (sampleInter.happened && !sampleInter.m->hasEmission()) {
                    auto f_r = inter.m->eval(view2Pos, sampleDir, inter.normal.normalized());
                    float pdf = inter.m->pdf(view2Pos, sampleDir, inter.normal);
                    float cosOnObj = dotProduct(inter.normal, sampleDir);
                    if (pdf > EPSILON)
                        L_indir = castRay(shot, depth + 1) * f_r * cosOnObj / pdf / RussianRoulette;
                }
            }

            break;
        }
    }
    

    return L_dir + L_indir;
    
}
