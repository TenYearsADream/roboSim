#pragma once

#include "Line.hh"
#include "../Config.hh"
#include "../Defines.hh"

class Environment : public Drawable {
public:
  std::vector<Line> lines;
  Ground target_ground;
  Ground ground;
  std::string name;

  Environment(Ground _target_ground) : target_ground(_target_ground), ground(_target_ground) {}

  void initTargetWithPolygons(const std::vector<Polygon>& _roomPolygons, float _robotRadius, std::string _name) {
    for (size_t i = 0; i < _roomPolygons.size(); i++) {
      std::vector<Line> daLines = _roomPolygons[i].toLines();
      lines.insert(lines.end(), daLines.begin(), daLines.end());
    }

    target_ground.markGroundTypePolygons(_roomPolygons);
    target_ground.transferGroundType(gt_probably_reachable_polygon, gt_visited);
    target_ground.markGridPointsNearPolygonsAsWalls(_roomPolygons, _robotRadius * 1.1f);

    name = _name;
  }

  float percentage() {
    return ground.compareToTarget(&target_ground);
  }

  std::string getName() {
    return name;
  }

  float nearestDistanceToWall(const Vec& _p) {
    float minDist = 10000000;
    for (size_t i = 0; i < lines.size(); i++) {
      minDist = std::min(minDist, lines[i].distanceToPoint(_p));
    }
    return minDist;
  }

  void reset() {
    ground.fill(gt_unknown);
  }

  bool intersectionWithLine(const Line& _line, Vec& _intersection, float _distance = 0) {
    std::vector<float> vector_of_s;
    for (size_t i = 0; i < lines.size(); i++) {
      float s, t;
      if (_distance == 0) {
        bool ok = intersect(_line, lines[i], s, t);
        if (ok) {
          vector_of_s.push_back(s);
        }
      }
      else {
        for (int left = 0; left < 2; left++) {
          intersect(_line, lines[i].parallelLineInDistance(_distance, left == 0), s, t);
          if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
            vector_of_s.push_back(s);

          //DEBUG_VAR(left == 0);
          std::vector<float> intersections_s = _line.intersectWithCircle_s(left == 0 ? lines[i].getStart() : lines[i].getEnd(), _distance);
          vector_of_s.insert(vector_of_s.end(), intersections_s.begin(), intersections_s.end());
        }
      }
      //DEBUG_VEC(vector_of_s);
    }

    if (vector_of_s.size()) {
      std::sort(vector_of_s.begin(), vector_of_s.end());
      float s = vector_of_s[0]/* - 0.01f*/;
      _intersection = _line.getStart() + (_line.getEnd() - _line.getStart()) * s;
      //DEBUG_VAR(_intersection);
    }
    //DEBUG_VAR(vector_of_s.size());
    return vector_of_s.size() > 0;
  }

  bool old_intersectionWithLine(const Line& _line, Vec& _intersection, float _distance = 0, const Vec& _farPoint = Vec()) {
    std::vector<float> vector_of_s;
    for (size_t i = 0; i < lines.size(); i++) {
      float s, t;
      if (_distance == 0) {
        bool ok = intersect(_line, lines[i], s, t);
        if (ok) {
          vector_of_s.push_back(s);
        }
      }
      else {
        intersect(_line, lines[i].parallelLineInDistance(_distance, _farPoint), s, t);

        float minDistance = fmin(_line.distanceToPoint(lines[i].getEnd()), _line.distanceToPoint(lines[i].getStart()));
        if (i == 0) {
          //DEBUG_VAR(s);
          //DEBUG_VAR(t);
          //DEBUG_VAR(_line.getStart());
          //DEBUG_VAR(_line.getEnd());
          //DEBUG_VAR(lines[i].getStart());
          //DEBUG_VAR(lines[i].getEnd());
          //DEBUG_VAR(_line.getStart() + (_line.getEnd() - _line.getStart())*s);
          //DEBUG_VAR(lines[i].distanceToPoint(_line.getStart() + (_line.getEnd() - _line.getStart()) * s));
          //DEBUG_VAR(_distance);
          //DEBUG_VAR(_line.distanceToPoint(lines[i].getEnd()));
          //DEBUG_VAR(_line.distanceToPoint(lines[i].getStart()));

          //DEBUG_VAR(minDistance);
        }

        if (s >= 0 && s <= 1) { // lines[i].distanceToPoint(_line.getStart() + (_line.getEnd() - _line.getStart()) * t)
          if (t >= 0 && t <= 1)
            vector_of_s.push_back(s);
          else if (minDistance <= _distance)
            vector_of_s.push_back(s);
        }
      }
    }

    if (vector_of_s.size()) {
      std::sort(vector_of_s.begin(), vector_of_s.end());
      float s = vector_of_s[0]/* - 0.01f*/;
      // _intersection = Vec(_line.getStart().x + s * (_line.getEnd().x - _line.getStart().x), _line.getStart().y + s * (_line.getEnd().y - _line.getStart().y));
      _intersection = _line.getStart() + (_line.getEnd() - _line.getStart()) * s;
      //DEBUG_VAR(_intersection);
    }
    std::cout << "vector_of_s.size() = " << vector_of_s.size() << std::endl;
    return vector_of_s.size() > 0;
  }

  bool intersectionWithMovement(const Vec& _pos, const float _dist, const float _angle, const float _radius, float& _moveableDistance) {
    _moveableDistance = _dist;
    Vec target = _pos.withAddedPolarCoordinates(_dist, _angle);

    Vec intersection;
    bool intersected = intersectionWithLine(Line(_pos, target), intersection, _radius);

    float distance = _pos.dist(intersection);
    if (intersected) {
      _moveableDistance = fmax(0, fmin(_moveableDistance, distance) - 0.01f);
    }
    return intersected;
  }

  // bool intersectionWithMovement(const Vec& _pos, const float _dist, const float _angle, const float _radius, float& _moveableDistance) {
  //   Vec frontPoint = _pos.withAddedPolarCoordinates(_radius, _angle);
  //   Vec frontTarget = _pos.withAddedPolarCoordinates(_dist, _angle);
  //   Vec rightPoint = _pos.withAddedPolarCoordinates(_radius, _angle + M_PI / 2.0f);
  //   Vec rightTarget = rightPoint.withAddedPolarCoordinates(_dist, _angle);
  //   Vec leftPoint = _pos.withAddedPolarCoordinates(_radius, _angle - M_PI / 2.0f);
  //   Vec leftTarget = leftPoint.withAddedPolarCoordinates(_dist, _angle);

  //   // Vec intersectionFront, intersectionRight, intersectionLeft;

  //   // bool intersectedFront = intersectionWithLine(Line(frontPoint, frontTarget), intersectionFront);
  //   // bool intersectedRight = intersectionWithLine(Line(rightPoint, rightTarget), intersectionRight);
  //   // bool intersectedLeft = intersectionWithLine(Line(leftPoint, leftTarget), intersectionLeft);

  //   // float distanceFront = frontPoint.dist(intersectionFront);
  //   // float distanceRight = rightPoint.dist(intersectionRight);
  //   // float distanceLeft = leftPoint.dist(intersectionLeft);

  //   // if (intersectedFront && distanceFront <= distanceRight && distanceFront <= distanceLeft)
  //   //   _moveableDistance = distanceFront;

  //   // else if (intersectedRight && distanceRight <= distanceFront && distanceFront <= distanceLeft)
  //   //   _moveableDistance = distanceRight;

  //   // else if (intersectedLeft && distanceLeft <= distanceFront && distanceFront <= distanceRight)
  //   //   _moveableDistance = distanceLeft;

  //   // else
  //   //   _moveableDistance = _dist;

  //   std::vector<Vec> startingPoints;

  //   int halfNumPoints = 10;
  //   for (int i = -halfNumPoints; i <= halfNumPoints; i++)
  //     startingPoints.push_back(_pos.withAddedPolarCoordinates(_radius, _angle + M_PI / 2.0f * ((float)i / (float)halfNumPoints)));

  //   _moveableDistance = _dist;
  //   bool intersectedAtAnyPoint = false;

  //   for (size_t i = 0; i < startingPoints.size(); i++) {
  //     Vec startingPoint = startingPoints[i];
  //     Vec target = startingPoint.withAddedPolarCoordinates(_dist, _angle);

  //     Vec intersection;
  //     bool intersected = intersectionWithLine(Line(startingPoint, target), intersection);

  //     float distance = startingPoint.dist(intersection);
  //     if (intersected) {
  //       intersectedAtAnyPoint = true;
  //       _moveableDistance = fmax(0, fmin(_moveableDistance, distance));
  //     }
  //   }

  //   return intersectedAtAnyPoint;
  // }

  Vec moveAsFarAsPossible(const Vec& _from, const Vec& _to, float _radius, bool& _collision) {
    Vec intersection;
    _collision = intersectionWithLine(Line(_from, _to), intersection);
    if (_collision) {
      return intersection;
    }
    return _to;
  }

  void draw() {
    GLUtils::drawLines(Config::color_lines_wall, lines);
  }

};