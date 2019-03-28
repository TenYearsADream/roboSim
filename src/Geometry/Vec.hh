#pragma once

class Vec {
public:
  float x, y;
  Vec() : x(0), y(0) {}
  Vec(float _x, float _y) : x(_x), y(_y) {}
  Vec(std::pair<float, float> _p) : x(_p.first), y(_p.second) {}
  std::pair<float, float> getPair() {
    return std::make_pair(x, y);
  }
  float dist(const Vec& v) const {
    return sqrt((x - v.x) * (x - v.x) + (y - v.y) * (y - v.y));
  }
  float length() const {
    return sqrt(x * x + y * y);
  }
  void addPolarCoordinates(int _radius, int _angle) {
    x = x + _radius * sin(_angle);
    y = y + _radius * cos(_angle);
  }

  Vec withAddedPolarCoordinates(float _radius, float _angle) const {
    float newx = x + _radius * sin(_angle);
    float newy = y + _radius * cos(_angle);
    return Vec(newx, newy);
  }

  float cross(Vec _v) const {
    return x * _v.y - y * _v.x;
  }

  float dot(Vec _v) const {
    return x * _v.x + y * _v.y;
  }

  float angleToXAxis() const {
    return atan2(y, x);
  }

  friend Vec operator+(Vec lhs, const Vec& rhs) {
    return Vec(lhs.x + rhs.x, lhs.y + rhs.y);
  }
  friend Vec operator-(Vec lhs, const Vec& rhs) {
    return Vec(lhs.x - rhs.x, lhs.y - rhs.y);
  }
  friend Vec operator*(Vec lhs, const float& factor) {
    return Vec(lhs.x * factor, lhs.y * factor);
  }
  friend Vec operator/(Vec lhs, const float& factor) {
    return Vec(lhs.x / factor, lhs.y / factor);
  }

  friend std::ostream& operator<<(std::ostream& os, const Vec& vec) {
    os << "(" << vec.x << "|" << vec.y << ")";
    return os;
  }
};

class Veci {
public:
  int x, y;
  Veci() : x(0), y(0) {}
  Veci(int _x, int _y) : x(_x), y(_y) {}

  Veci leftHandPerpendicular() {
    return Veci(-y, x);
  }

  Veci rightHandPerpendicular() {
    return Veci(y, -x);
  }

  friend Veci operator+(Veci lhs, const Veci& rhs) {
    return Veci(lhs.x + rhs.x, lhs.y + rhs.y);
  }
  friend Veci operator-(Veci lhs, const Veci& rhs) {
    return Veci(lhs.x - rhs.x, lhs.y - rhs.y);
  }
  friend bool operator==(Veci lhs, const Veci& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }

  friend std::ostream& operator<<(std::ostream& os, const Veci& veci) {
    os << "(" << veci.x << "|" << veci.y << ")";
    return os;
  }
};



