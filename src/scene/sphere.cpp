#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
    double a = dot(r.d, r.d);
    double b = dot(2.0 * (r.o - this->o), r.d);
    double c = dot((r.o - this->o), (r.o - this->o)) - this->r2;
    double in_sq = pow(b, 2.0) - 4 * a * c;
    if (in_sq < 0) {
        return false;
    }
    double t_pos = (-1.0 * b + sqrt(in_sq)) / 2 * a;
    double t = (-1.0 * b - sqrt(in_sq)) / 2 * a;
    if (t < 0) {
        t = t_pos;
    }
    bool result = t >= r.min_t and t <= r.max_t;
    if (result) {
        r.max_t = t;
    }
    return result;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    if (has_intersection(r)) {
        i->t = r.max_t;
        Vector3D intersect = r.o + r.max_t * r.d;
        i->n = (intersect - this->o) / this->r ;
        i->primitive = this;
        i->bsdf = this->get_bsdf();
        return true;
    }
    return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
