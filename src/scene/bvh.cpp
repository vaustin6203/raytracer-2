#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  int size = 0;
  BBox box = BBox();
  Vector3D avecentroid = Vector3D(0, 0, 0);
  for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      box.expand(bb);
      size += 1;
      avecentroid += bb.centroid();
  }
  BVHNode *node = new BVHNode(box);
  if (size <= max_leaf_size) {
      node->start = start;
      node->end = end;
      return node;
  }
  Vector3D diff = (box.max - box.min);
  int index = 0;
  double ave = 0;
  if(diff.x >= diff.y && diff.x >= diff.z) {
      ave = avecentroid.x / size;
  } else if (diff.y > diff.x && diff.y >= diff.z) {
      index = 1;
      ave = avecentroid.y / size;
  } else {
      index = 2;
      ave = avecentroid.z / size;
  }
  std::vector<Primitive *> l;
  std::vector<Primitive *> r;
  for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      if(bb.centroid()[index] <= ave) {
          l.push_back(*p);
      } else {
          r.push_back(*p);
      }
  }
  size_t l_size = l.size();
  size_t r_size = r.size();
  if (l_size == 0 || r_size == 0) {
      node->start = start;
      node->end = end;
      return node;
  } else {
      std::vector<Primitive *>::iterator curr = start;
      std::vector<Primitive *>::iterator l_end;
      for (int i = 0; i < l_size; i++) {
          *curr = l[i];
          curr++;
      }
      l_end = start + l_size;
      for (int j = 0; j < r_size; j++) {
          *curr = r[j];
          curr++;
      }
      node->start = start;
      node->end = end;
      node->l = construct_bvh(start, l_end, max_leaf_size);
      node->r = construct_bvh(l_end, end, max_leaf_size);
  }
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  double miT = ray.min_t;
  double maT = ray.max_t;
  total_isects++;
  if (!node->bb.intersect(ray, miT, maT)) {
        return false;
  }

  for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
          return true;
      }
  }
  return false;
  /**
  for (auto p : primitives) {
      total_isects++;
      if (p->has_intersection(ray))
        return true;
  }
    return false;
    */
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    total_isects++;
    double miT = ray.min_t;
    double maT = ray.max_t;
    if (!node->bb.intersect(ray, miT, maT)) {
        return false;
    }

    if (node->isLeaf()) {
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
            total_isects++;
            hit = (*p)->intersect(ray, i) || hit;
        }
        return hit;
    }

    bool l = intersect(ray, i, node->l);
    bool r = intersect(ray, i, node->r);
    return l || r;
   /**
  bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;
    */
}

} // namespace SceneObjects
} // namespace CGL
