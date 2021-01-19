#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  double tx1 = (this->min.x - r.o.x) / r.d.x;
  double tx2 = (this->max.x - r.o.x) / r.d.x;
  double ty1 = (this->min.y - r.o.y) / r.d.y;
  double ty2 = (this->max.y - r.o.y) / r.d.y;
  double tz1 = (this->min.z - r.o.z) / r.d.z;
  double tz2 = (this->max.z - r.o.z) / r.d.z;

  double miX = tx1;
  double maX = tx2;

  if (tx2 < tx1) {
      miX = tx2;
      maX = tx1;
  }

  double miY = ty1;
  double maY = ty2;

  if (ty2 < ty1) {
      miY = ty2;
      maY = ty1;
  }

  double miZ = tz1;
  double maZ = tz2;

  if (tz2 < tz1) {
      miZ = tz2;
      maZ = tz1;
  }

  double mi = miX;
  double ma = maX;

  if (miY >= miX && miY >= miZ) {
      mi = miY;
  } else if (miZ > miX && miZ > miY) {
      mi = miZ;
  }

  if (maY <= maX && maY <= maZ) {
      ma = maY;
  } else if (maZ < maX && maZ < maY) {
      ma = maZ;
  }

  if (mi <= ma && t1 > mi) {
      t0 = mi;
      t1 = ma;
      return true;
  }
  return false;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
