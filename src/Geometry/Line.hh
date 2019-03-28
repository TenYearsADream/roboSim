#pragma once

#include "Vec.hh"
#include "../Utils/MathUtils.hh"
#include <iostream>
#include "../Defines.hh"

struct Line {
  float x1, x2, y1, y2;
  Line(float _x1, float _y1, float _x2, float _y2) : x1(_x1), x2(_x2), y1(_y1), y2(_y2) {};
  Line(Vec _p1, Vec _p2) : x1(_p1.x), x2(_p2.x), y1(_p1.y), y2(_p2.y) {};
  Vec getStart() const {
    return Vec(x1, y1);
  }
  Vec getEnd() const {
    return Vec(x2, y2);
  }
  void setStart(Vec _start) {
    x1 = _start.x;
    y1 = _start.y;
  }
  void setEnd(Vec _end) {
    x2 = _end.x;
    y2 = _end.y;
  }
  float length() const {
    return getStart().dist(getEnd());
  }
  float infiniteLine_distanceToPoint(const Vec& _p) const {
    return fabs((getEnd() - getStart()).cross(getStart() - _p)) / (getEnd() - getStart()).length();
  }
  float distanceToPoint(const Vec& _p) const {
    float t = (_p - getStart()).dot(getEnd() - getStart()) / length() / length(); // Divide two times by length!
    // std::cout << "t=" << t << ", " << "distanceToPoint: " << (*this) << "/" << _p << " = " << (getEnd() - getStart()).cross(getStart() - _p) / (getEnd() - getStart()).length() << std::endl; // Output is wrong, see if statement

    if (t >= 1)
      return _p.dist(getEnd());
    else if (t <= 0)
      return _p.dist(getStart());

    return infiniteLine_distanceToPoint(_p);
  }

  bool isLeft(const Vec& _p) const {
    return ((getEnd().x - getStart().x) * (_p.y - getStart().y) - (getEnd().y - getStart().y) * (_p.x - getStart().x)) > 0;
  }

  Line parallelLineInDistance(const float& _distance, const Vec& _farPoint = Vec()) const {
    if (_distance == 0)
      return *this;

    Vec v = Vec(y2 - y1, x1 - x2);

    // Ensure v is pointing in direction of _farPoint
    if (isLeft(getStart() + v) != isLeft(_farPoint))
      v = v * -1.0f;

    // Normalize
    v = v / v.length();

    return Line(getStart() + v * _distance, getEnd() + v * _distance);
  }

  Line parallelLineInDistance(const float& _distance, bool _left) const {
    Vec v = Vec(y2 - y1, x1 - x2);
    return parallelLineInDistance(_distance, _left ? getStart() + v : getStart() - v);
  }

  friend std::ostream& operator<<(std::ostream& os, const Line& line) {
    os << "(" << line.x1 << "|" << line.y1 << ")--(" << line.x2 << "|" << line.y2 << ")";
    return os;
  }

  std::vector<Vec> infiniteLine_intersectWithCircle(const Vec& _pos, const float& _radius) const { // http://mathworld.wolfram.com/Circle-LineIntersection.html
    Vec d = getEnd() - getStart();
    float D = (x1 - _pos.x) * (y2 - _pos.y) - (x2 - _pos.x) * (y1 - _pos.y);
    float Delta = _radius * _radius * d.length() * d.length() - D * D;

    //DEBUG_VAR(D);
    //DEBUG_VAR(Delta);

    if (Delta < 0)
      return std::vector<Vec>();
    else if (Delta == 0)
      return { _pos + Vec(D * d.y, -D * d.x) / d.length() / d.length() };
    else
      return { _pos + Vec(D * d.y + MathUtils::signStar(d.y) * d.x * sqrt(Delta), -D * d.x + fabs(d.y) * sqrt(Delta)) / d.length() / d.length(),
               _pos + Vec(D * d.y - MathUtils::signStar(d.y) * d.x * sqrt(Delta), -D * d.x - fabs(d.y) * sqrt(Delta)) / d.length() / d.length() };
  }

  std::vector<Vec> removePointsNotOnLine(const std::vector<Vec>& _points) const {
    float minX = fmin(x1, x2);
    float maxX = fmax(x1, x2);
    float minY = fmin(y1, y2);
    float maxY = fmax(y1, y2);

    //DEBUG_VEC(_points);
    std::vector<Vec> points;
    for (size_t i = 0; i < _points.size(); i++) {
      if (minX == maxX) {
        if (_points[i].y >= minY && _points[i].y <= maxY)
          points.push_back(_points[i]);
      }
      else {
        if (_points[i].x >= minX && _points[i].x <= maxX)
          points.push_back(_points[i]);
      }
    }
    //DEBUG_VEC(points);
    return points;
  }

  std::vector<Vec> intersectWithCircle(const Vec& _pos, const float& _radius) const {
    //DEBUG_VAR(_pos);
    //DEBUG_VAR(_radius);
    std::vector<Vec> intersections = infiniteLine_intersectWithCircle(_pos, _radius);
    //DEBUG_VEC(intersections);
    return removePointsNotOnLine(infiniteLine_intersectWithCircle(_pos, _radius));
  }

  std::vector<float> intersectWithCircle_s(const Vec& _pos, const float& _radius) const {
    float minX = fmin(x1, x2);
    float maxX = fmax(x1, x2);
    float minY = fmin(y1, y2);
    float maxY = fmax(y1, y2);

    std::vector<Vec> intersections = intersectWithCircle(_pos, _radius);
    std::vector<float> vector_of_s;
    for (size_t i = 0; i < intersections.size(); i++) {
      if (minX == maxX)
        vector_of_s.push_back((intersections[i].y - minY) / (maxY - minY));
      else
        vector_of_s.push_back((intersections[i].x - minX) / (maxX - minX));
    }
    return vector_of_s;
  }
};

static bool intersect(const Line& line1, const Line& line2, float& u, float& t) {
  Vec s1(line1.x1, line1.y1);
  Vec d1(line1.x2 - line1.x1, line1.y2 - line1.y1);
  Vec s2(line2.x1, line2.y1);
  Vec d2(line2.x2 - line2.x1, line2.y2 - line2.y1);

  u = ((s1.y - s2.y) * d2.x - (s1.x - s2.x) * d2.y) / (d1.x * d2.y - d1.y * d2.x);
  t = ((s2.y - s1.y) * d1.x - (s2.x - s1.x) * d1.y) / (d2.x * d1.y - d2.y * d1.x);
  return u >= 0 && u <= 1 && t >= 0 && t <= 1;
}