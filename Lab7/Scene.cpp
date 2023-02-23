//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <vector>

void Scene::buildBVH() {
  printf(" - Generating BVH...\n\n");
  this->bvh = new BVHAccel(objects);
}

Intersection Scene::intersect(const Ray &ray) const {
  return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
  float emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
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

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) {
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
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  // TO DO Implement Path Tracing Algorithm here
  auto intersection = intersect(ray);
  if (!intersection.happened) {
    return Vector3f();
  }

  if (intersection.m->hasEmission()) {
    return intersection.m->getEmission();
  }

  Vector3f light_dir;

  Intersection light_intersection;
  float pdf_light;
  sampleLight(light_intersection, pdf_light);
  auto N = intersection.normal.normalized();
  auto &p = intersection.coords;
  auto &x = light_intersection.coords;
  auto ws = (x - p).normalized();
  auto ws_distance = (x - p).norm();

  auto ws_ray = Ray(p, ws);
  auto ws_intersection = intersect(ws_ray);
  if (ws_intersection.distance - ws_distance > -EPSILON) {
    auto &emit = light_intersection.emit;
    auto NN = light_intersection.normal.normalized();

    light_dir =
        emit * intersection.m->eval(ray.direction, ws_ray.direction, N) *
        dotProduct(ws_ray.direction, N) * dotProduct(-ws_ray.direction, NN) /
        (ws_distance * ws_distance) / pdf_light;
  }

  if (get_random_float() > RussianRoulette) {
    return light_dir;
  }

  Vector3f light_indir;

  auto new_dir = intersection.m->sample(ray.direction, N).normalized();
  auto new_ray = Ray(p, new_dir);
  auto new_intersection = intersect(new_ray);
  if (new_intersection.happened && !new_intersection.m->hasEmission()) {
    light_indir = castRay(new_ray, depth + 1) *
                  intersection.m->eval(ray.direction, new_ray.direction, N) *
                  dotProduct(new_ray.direction, N) /
                  intersection.m->pdf(ray.direction, new_ray.direction, N) /
                  RussianRoulette;
  }

  return light_dir + light_indir;
}