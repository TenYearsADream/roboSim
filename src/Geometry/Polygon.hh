#pragma once

#include "Vec.hh"
#include "Line.hh"
// #include "../Utils/GLUtils.hh"

class Polygon : public std::vector<Vec> {
public:
  bool closed;
  bool hasCycle() {
    if (this->size() < 3)
      return false;
    for (size_t i = 0; i < size(); i++)
      for (size_t j = 0; j < size(); j++)
        if (i != j && this->operator[](0).dist(this->operator[](i)) < 0.001)
          return true;
    return false;
  }

  // void draw(GLUtils::Color _color) {
  //   GLUtils::drawPolygon(_color, *this);
  // }

  std::vector<Line> toLines() const {
    std::vector<Line> lines;
    for (size_t i = 0; i < size(); i++) {
      lines.push_back(Line((*this)[i], (*this)[(i + 1) % size()]));
    }
    return lines;
  }

  void addPointRelative(const Vec& _p) {
    Vec a = this->back();
    this->push_back(this->back() + _p);
    assert(this->back().x != a.x || this->back().y != a.y);
  }

  static void addPoints_livingRoomKitchen(Polygon& roomPoints) {
    roomPoints.addPointRelative(Vec(2.05, 0));
    roomPoints.addPointRelative(Vec(0, 1.09));
    roomPoints.addPointRelative(Vec(0.14, 0));
    roomPoints.addPointRelative(Vec(0, 3.30));
    roomPoints.addPointRelative(Vec(-3.47, 0));
    roomPoints.addPointRelative(Vec(0, -2.85));
    roomPoints.addPointRelative(Vec(0, -0.11));
    roomPoints.addPointRelative(Vec(-0.4, 0));
    roomPoints.addPointRelative(Vec(0, 0.22));
    roomPoints.addPointRelative(Vec(-0.05, 0));
    roomPoints.addPointRelative(Vec(0, 0.58));
    roomPoints.addPointRelative(Vec(-0.39, 0));
    roomPoints.addPointRelative(Vec(0, 2.13));
    roomPoints.addPointRelative(Vec(-1.65, 0));
    roomPoints.addPointRelative(Vec(0, -0.63));
    roomPoints.addPointRelative(Vec(0.18, 0));
    roomPoints.addPointRelative(Vec(0, -2.57));
    roomPoints.addPointRelative(Vec(1.05, 0));
    roomPoints.addPointRelative(Vec(0, -0.51));
    roomPoints.addPointRelative(Vec(0.72, 0));
    roomPoints.addPointRelative(Vec(0.63, -0.65));
  }

  static void addPoints_secondRoom(Polygon& roomPoints) {
    roomPoints.addPointRelative(Vec(0, -0.88));
    roomPoints.addPointRelative(Vec(-0.10, 0));
    roomPoints.addPointRelative(Vec(0, -2.69));
    roomPoints.addPointRelative(Vec(0.52, 0));

    roomPoints.addPointRelative(Vec(0.03, -0.22));
    roomPoints.addPointRelative(Vec(-0.03, -0.22));


    roomPoints.addPointRelative(Vec(-0.52, 0));
    roomPoints.addPointRelative(Vec(0, -0.65));
    roomPoints.addPointRelative(Vec(0.52, 0));
    roomPoints.addPointRelative(Vec(0, -0.06));
    roomPoints.addPointRelative(Vec(-0.08, 0));
    roomPoints.addPointRelative(Vec(0, -0.19));
    roomPoints.addPointRelative(Vec(1.90, 0));
    roomPoints.addPointRelative(Vec(0, 0.80));
    roomPoints.addPointRelative(Vec(-0.56, 0));
    roomPoints.addPointRelative(Vec(0, 1.37));
    roomPoints.addPointRelative(Vec(0.56, 0));
    roomPoints.addPointRelative(Vec(0, 2.20));
    roomPoints.addPointRelative(Vec(-1.38, 0));
    roomPoints.addPointRelative(Vec(0, 0.48));
  }

  static void addFirstPoints_roomAndHall(Polygon& roomPoints) {
    roomPoints.push_back(Vec(0, 0));
    roomPoints.push_back(Vec(3.55, 0));
    roomPoints.push_back(Vec(3.55, 2.20));
    roomPoints.push_back(Vec(3.0, 2.20));
    roomPoints.push_back(Vec(3.0, 4.00));
    roomPoints.push_back(Vec(3.55, 4.00));
    roomPoints.push_back(Vec(3.55, 5.00));
    roomPoints.push_back(Vec(2.82, 5.00));

    roomPoints.push_back(Vec(2.30, 4.50));
    roomPoints.push_back(Vec(2.28, 4.53));
    roomPoints.push_back(Vec(2.82, 5.00));

    roomPoints.push_back(Vec(2.82, 5.20));
    roomPoints.addPointRelative(Vec(0.97, 0));
    roomPoints.addPointRelative(Vec(0, -0.32));
  }

  static void addSecondPoints_roomAndHall(Polygon& roomPoints) {
    roomPoints.addPointRelative(Vec(0, 0.32));
    roomPoints.addPointRelative(Vec(1.57, 0));

    roomPoints.addPointRelative(Vec(0, 1.40));
    roomPoints.addPointRelative(Vec(-2.20, 0));
    roomPoints.addPointRelative(Vec(0, 0.12));

    // roomPoints.push_back(Vec(7.0, 6.60));

    // roomPoints.push_back(Vec(4.80, 6.60));
    // roomPoints.push_back(Vec(4.80, 6.72));
  }
  static void addThirdPoints_roomAndHall(Polygon& roomPoints) {
    roomPoints.addPointRelative(Vec(0, -0.12));
    roomPoints.addPointRelative(Vec(-1.32, 0));
    // roomPoints.push_back(Vec(3.96, 6.72));
    // roomPoints.push_back(Vec(3.96, 6.60));

    roomPoints.push_back(Vec(1.0, 6.60));
    roomPoints.push_back(Vec(1.0, 5.20));

    roomPoints.push_back(Vec(2.0, 5.20));

    roomPoints.push_back(Vec(2.0, 5.00));
    roomPoints.push_back(Vec(0.65, 5.00));
    roomPoints.push_back(Vec(0.65, 4.26));
    roomPoints.push_back(Vec(0.24, 4.26));
    roomPoints.push_back(Vec(0.24, 2.91));
    roomPoints.push_back(Vec(0, 2.91));
  }

  static Polygon generateRoomExtended() {
    Polygon roomPoints;

    addFirstPoints_roomAndHall(roomPoints);
    addPoints_secondRoom(roomPoints);
    addSecondPoints_roomAndHall(roomPoints);
    addPoints_livingRoomKitchen(roomPoints);
    addThirdPoints_roomAndHall(roomPoints);

    return roomPoints;
  }

  static Polygon generateRoom() {
    Polygon roomPoints;

    addFirstPoints_roomAndHall(roomPoints);
    roomPoints.addPointRelative(Vec(0.86, 0));
    addSecondPoints_roomAndHall(roomPoints);
    roomPoints.addPointRelative(Vec(-0.84, 0));
    addThirdPoints_roomAndHall(roomPoints);

    return roomPoints;
  }

  static Polygon generatePillar() {
    Polygon roomPoints;

    roomPoints.push_back(Vec(1, 1));
    roomPoints.push_back(Vec(2, 1));
    roomPoints.push_back(Vec(2, 2));
    roomPoints.push_back(Vec(1, 2));

    return roomPoints;
  }
};