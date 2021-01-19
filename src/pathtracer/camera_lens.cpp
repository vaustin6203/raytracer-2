#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {
  // Part 2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
  double camera_x = (x - 0.5) * 2.0 * tan(((hFov * M_PI) / 180) * 0.5);
  double camera_y = (y - 0.5) * 2.0 * tan(((vFov * M_PI) / 180) * 0.5);
  Vector3D red_direction = Vector3D(camera_x, camera_y, -1);
  double val = lensRadius * sqrt(rndR);
  Vector3D pLens = Vector3D(val * cos(rndTheta), val * sin(rndTheta), 0);
  Vector3D pFocus = red_direction * focalDistance - pLens;
  Vector3D direction = c2w * pFocus;
  direction.normalize();
  Ray ray = Ray(c2w * pLens + pos, direction);
  ray.min_t = nClip;
  ray.max_t = fClip;
  return ray;
}


} // namespace CGL
