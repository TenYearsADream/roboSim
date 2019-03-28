#pragma once

#include <vector>
#include <iostream>
#include <sstream>
#include <iterator>
#include "../Geometry/Vec.hh"
#include "../Defines.hh"

namespace MathUtils {

typedef Eigen::Vector3f Vec3f;

static bool almostEqual(const float& a, const float& b, const float& epsilon) {
  return fabs(a - b) < epsilon;
}

static float sign(Vec p1, Vec p2, Vec p3) {
  return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

static bool linearDependent(Vec v1, Vec v2, Vec v3) {
  return (v2.x - v1.x) / (v2.y - v1.y) == (v3.x - v1.x) / (v3.y - v1.y);
}

static bool PointInTriangle(Vec pt, Vec v1, Vec v2, Vec v3) {
  bool b1, b2, b3;

  b1 = sign(pt, v1, v2) < 0.0f;
  b2 = sign(pt, v2, v3) < 0.0f;
  b3 = sign(pt, v3, v1) < 0.0f;

  return (!linearDependent(v1, v2, v3) && ((b1 == b2) && (b2 == b3)));
}

static float triangleArea(Vec a, Vec b, Vec c) {
  return fabs((a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y)) / 2.0f);
}

static bool in(int _w, std::vector<int> _v) {
  for (size_t i = 0; i < _v.size(); i++)
    if (_v[i] == _w)
      return true;
  return false;
}

static float signStar(const float& _x) {
  return _x < 0 ? -1 : 1;
}

static float fmodPosNeg(float _a, float _b) {
  float r = fmod(_a, _b);
  return r < 0 ? r + _b : r;
}

static Eigen::Vector3f linePlaneIntersection(Vec3f _planeBase, Vec3f _planex2, Vec3f _planex3, Vec3f _lineStart, Vec3f _lineEnd) { // http://mathworld.wolfram.com/Line-PlaneIntersection.html
  Eigen::Matrix4f mat1;
  mat1 << 1, 1, 1, 1,
       _planeBase.x(), _planex2.x(), _planex3.x(), _lineStart.x(),
       _planeBase.y(), _planex2.y(), _planex3.y(), _lineStart.y(),
       _planeBase.z(), _planex2.z(), _planex3.z(), _lineStart.z();
  Eigen::Matrix4f mat2;
  mat2 << 1, 1, 1, 0,
       _planeBase.x(), _planex2.x(), _planex3.x(), _lineEnd.x() - _lineStart.x(),
       _planeBase.y(), _planex2.y(), _planex3.y(), _lineEnd.y() - _lineStart.y(),
       _planeBase.z(), _planex2.z(), _planex3.z(), _lineEnd.z() - _lineStart.z();
  float t = -1.0f * mat1.determinant() / mat2.determinant();
  float x = _lineStart.x() + (_lineEnd.x() - _lineStart.x()) * t;
  float y = _lineStart.y() + (_lineEnd.y() - _lineStart.y()) * t;
  float z = _lineStart.z() + (_lineEnd.z() - _lineStart.z()) * t;
  return Eigen::Vector3f(x, y, z);
}

}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if (!v.empty()) {
    out << '['; // No ostream_iterator in std
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}