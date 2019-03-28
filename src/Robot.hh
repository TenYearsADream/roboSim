#pragma once

#include "Geometry/Vec.hh"
#include "Geometry/Line.hh"
#include "Geometry/Polygon.hh"
#include "Geometry/Ground.hh"
#include "Geometry/Environment.hh"
#include "Utils/MathUtils.hh"
#include "Utils/GLUtils.hh"
#include "Visualization/Drawable.hh"
#include "Config.hh"
#include "Defines.hh"

class Robot : public Drawable {
private:
  float alpha;
  Vec position;
  float radius = 0.15;
  float radius_search = 1.5;
  float speed = 0.05; // m/sekunde (ca. 5cm pro sekunde?)

  bool isCurrentlyCollided = false;
  unsigned int collisions = 0;
  size_t milliseconds_simulated = 0;

  bool dirty = false;
  bool finished = false;

public:
  Environment* environment;

  virtual void draw_specific() = 0;
  virtual void simulateOneTenthSecond_specific() = 0;
  virtual void handleKeys(unsigned char key, int x, int y) = 0;
  virtual std::string getName() = 0;

  Robot(Environment* _environment, Vec _position, float _alpha) : environment(_environment), position(_position), alpha(MathUtils::fmodPosNeg(_alpha, 2 * M_PI)) {}

  void setDirty() { dirty = true; }
  void setFinished() { finished = true; }
  bool isFinished() { return finished; }
  Vec currentPosition() const { return position; }
  void setPosition(Vec _newpos) { position = _newpos; setDirty(); }
  float getAlpha() const { return MathUtils::fmodPosNeg(alpha, 2 * M_PI); }
  void setAlpha(float _alpha) { alpha = MathUtils::fmodPosNeg(_alpha, 2 * M_PI); }
  void rotate(float _deltaAlpha) { alpha = MathUtils::fmodPosNeg(alpha + _deltaAlpha, 2 * M_PI); }
  void rotateDegree(float _deltaAlpha) { alpha = MathUtils::fmodPosNeg(alpha + _deltaAlpha / 180.0f * M_PI, 2 * M_PI); }
  float getRadius() const { return radius; }
  float getRadiusSearch() const { return radius_search; }
  float getSpeed() const { return speed; }
  size_t getMillisecondsSimulated() const { return milliseconds_simulated; }
  unsigned int getCollisions() const { return collisions; }

  float dist(Vec _pos) { return position.dist(_pos); }

  void draw() {
    GLUtils::drawCircle(Config::color_robo, currentPosition(), radius);
    GLUtils::drawLine(Config::color_robo, currentPosition(), currentPosition().withAddedPolarCoordinates(radius, getAlpha()));

    draw_specific();
  }

  void simulateOneTenthSecond() {
    simulateOneTenthSecond_specific();
    environment->ground.markVisitedInRadius(currentPosition(), radius);
    milliseconds_simulated += 100;
  }

  void simulateOneSecond() {
    for (int i = 0; i < 10; i++)
      simulateOneTenthSecond();
  }

  void moveForward() {
    if (isCurrentlyCollided) {
      if (environment->nearestDistanceToWall(currentPosition()) > radius) {
        isCurrentlyCollided = false;
      }
    }

    // position = currentPosition().withAddedPolarCoordinates(-0.01, getAlpha());

    float moveableDistance;
    bool collision = environment->intersectionWithMovement(currentPosition(), getSpeed() / 10.0f, getAlpha(), radius, moveableDistance);
    // DEBUG_VAR(moveableDistance);
    if (!isCurrentlyCollided && collision) {
      collisions++;
      isCurrentlyCollided = true;
    }

    if (moveableDistance > 0.001f) {
      Vec newpos = currentPosition().withAddedPolarCoordinates(moveableDistance, getAlpha());
      // DEBUG_VAR(newpos);
      if (environment->nearestDistanceToWall(newpos) > radius)
        position = newpos;
    }
  }

  float percentage() {
    return environment->percentage();
  }

  float frontDistance(bool& _intersected, Vec& _intersection) {
    Vec front = currentPosition().withAddedPolarCoordinates(radius_search, getAlpha());

    Line searchline(currentPosition(), front);

    _intersected = environment->intersectionWithLine(searchline, _intersection);

    return _intersected ? _intersection.dist(currentPosition()) : radius_search;
  }

  bool checkFrontIntersection(Vec& _intersection) {
    bool intersected;
    frontDistance(intersected, _intersection);
    return intersected;
  }

  struct PassedTime {
    size_t hours, minutes, seconds;
  };

  size_t hours_passed() {
    size_t hours = milliseconds_simulated / 1000 / 60 / 60;
    return hours;
  }

  size_t minutes_passed() {
    size_t hours = milliseconds_simulated / 1000 / 60 / 60;
    size_t minutes = milliseconds_simulated / 1000 / 60 - hours * 60;
    return minutes;
  }

  size_t seconds_passed() {
    size_t hours = milliseconds_simulated / 1000 / 60 / 60;
    size_t minutes = milliseconds_simulated / 1000 / 60 - hours * 60;
    size_t seconds = milliseconds_simulated / 1000 - hours * 60 * 60 - minutes * 60;
    return seconds;
  }

  PassedTime time_passed() {
    PassedTime passedTime;
    passedTime.hours = milliseconds_simulated / 1000 / 60 / 60;
    passedTime.minutes = milliseconds_simulated / 1000 / 60 - passedTime.hours * 60;
    passedTime.seconds = milliseconds_simulated / 1000 - passedTime.hours * 60 * 60 - passedTime.minutes * 60;
    return passedTime;
  }

};
