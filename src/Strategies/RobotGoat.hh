#pragma once

class RobotGoat : public Robot {
public:
  RobotGoat(Environment* _environment, Vec _position, float _alpha) : Robot(_environment, _position, _alpha) {}

  void handleKeys(unsigned char key, int x, int y) {

  }

  std::string getName() {
    return "Goat";
  }

  void simulateOneTenthSecond_specific() {
    if (getMillisecondsSimulated() / 1000 % (5) == 0)
      rotateDegree(rand() % 360);
    else
      moveForward();
  }

  void draw_specific() {
    environment->draw();
    environment->ground.draw();
  }
};