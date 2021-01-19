#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
  Intersection isect2;
  for (int i = 0; i < num_samples; i++) {
      Vector3D w_in = this->hemisphereSampler->get_sample();
      Vector3D direction = o2w * w_in;
      Vector3D origin = hit_p + (2.0 * direction * EPS_D);
      Ray ray = Ray(origin, direction);
      if (this->bvh->intersect(ray, &isect2)) {
          Spectrum f = isect.bsdf->f(w_out, w_in);
          Spectrum li = isect2.bsdf->get_emission();
          L_out += f * li * abs_cos_theta(w_in);
      }
  }
  return (L_out * 2.0 * M_PI) / num_samples;
}

Spectrum
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;

  for (SceneLight *light : scene->lights) {
      int num_samples;
      if (light->is_delta_light()) {
          num_samples = 1;
      } else {
          num_samples = ns_area_light;
      }
      Vector3D wi;
      float distToLight;
      float pdf;
      for (int i = 0; i < num_samples; i++) {
          Spectrum li = light->sample_L(hit_p, &wi, &distToLight, &pdf);
          Vector3D w_in = w2o * wi;
          //If light is not behind surface at hit point
          if (w_in.z >= 0) {
              Vector3D origin = hit_p + (2.0 * wi * EPS_D);
              Ray ray = Ray(origin, wi);
              ray.max_t = distToLight;
              Intersection inter2;
              if (!this->bvh->intersect(ray, &inter2)) {
                  Spectrum f = isect.bsdf->f(w_out, w_in);
                  L_out += (li * f * abs_cos_theta(w_in)) / pdf;
              }
          }
      }
      L_out = L_out / num_samples;
  }
  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
      return estimate_direct_lighting_hemisphere(r, isect);
  }
  return estimate_direct_lighting_importance(r, isect);
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Spectrum L_out;
  if (!isect.bsdf->is_delta()) {
      L_out += one_bounce_radiance(r, isect);
  }

  Vector3D w_in;
  float pdf;
  Spectrum li = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  //avoids dividing by 0
  if (pdf == 0) {
      return L_out;
  }
  bool russianRoulette = coin_flip(0.7);
  if (this->max_ray_depth == r.depth || (russianRoulette && r.depth >= 1)) {
      Vector3D direction = o2w * w_in;
      Vector3D origin = hit_p + (2.0 * direction * EPS_D);
      Ray ray = Ray(origin, direction);
      ray.depth = r.depth - 1;
      Intersection inter2;
      if (this->bvh->intersect(ray, & inter2)) {
          Spectrum moreBounce = at_least_one_bounce_radiance(ray, inter2);
          if (isect.bsdf->is_delta()) {
             moreBounce += zero_bounce_radiance(ray, inter2);
          }
          if (this->max_ray_depth == r.depth) {
              L_out += (w_in.z * li * moreBounce) / pdf;
          } else {
              L_out += ((w_in.z * li * moreBounce) / pdf) / 0.7;
          }
      }
  }
  return L_out;
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect))
    return L_out;

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  //return zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  return zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

  // TODO (Part 1.1):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Spectrum.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Spectrum spec = Spectrum(0, 0, 0);
  double width = sampleBuffer.w;
  double height = sampleBuffer.h;
  double s1 = 0;
  double s2 = 0;
  int num_converge = num_samples;
  if (num_samples == 1) {
      Vector2D sample = gridSampler->get_sample();
      sample.x = (sample.x + origin.x) / width;
      sample.y = (sample.y + origin.y) / height;
      Ray r = camera->generate_ray(sample.x, sample.y);
      //modify ray's depth to max_ray_depth
      r.depth = this->max_ray_depth;
      spec = PathTracer::est_radiance_global_illumination(r);
  } else {
      for (int i = 0; i < num_samples; i++) {
          if (i % samplesPerBatch == 0 && i) {
              double mean = s1 / (double) i;
              double var = (s2 - (pow(s1, 2.0) / i)) / ((double) (i - 1));
              double interval = 1.96 * sqrt(var / i);
              if (interval <= maxTolerance * mean) {
                  num_converge = i;
                  break;
              }
          }
          Vector2D sample = gridSampler->get_sample();
          sample.x = (sample.x + origin.x) / width;
          sample.y = (sample.y + origin.y) / height;
          Ray r = camera->generate_ray(sample.x, sample.y);
          //modify ray's depth to max_ray_depth
          r.depth = this->max_ray_depth;
          Spectrum temp = PathTracer::est_radiance_global_illumination(r);
          spec += temp;
          double ilm = temp.illum();
          s1 += ilm;
          s2 += ilm * ilm;
      }
  }
  spec = spec / num_converge;
  sampleBuffer.update_pixel(spec, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_converge;
}

} // namespace CGL