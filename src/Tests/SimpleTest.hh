#pragma once

#include "Test.hh"

using Json = nlohmann::json;

class SimpleTest : public Test {
public:
  unsigned int minutesToSimulate;
  unsigned int iterationsToSimulate;
  SimpleTest(Robot* _robot, unsigned int _minutesToSimulate = 10, unsigned int _iterationsToSimulate = 1) : Test(_robot), minutesToSimulate(_minutesToSimulate), iterationsToSimulate(_iterationsToSimulate) {}

  std::string getName_specific() const {
    return "simpleTest";
  }

  void runTest_specific() {
    data = Json::array();

    std::vector<Vec> poss = { Vec(0.5, 0.5), Vec(3.27, 0.27), Vec(5.84, 6.22), Vec(1.31, 6.24), Vec(3.06, 4.49)};
    if (robot->environment->getName() == "fullApartement")
      poss = { Vec(0.5, 0.5), Vec(4.88, 0.58), Vec(5.8, 6.95), Vec(0.91, 9.0), Vec(1.36, 5.69)};
    std::vector<float> alphas = { 0.1, 0.1, 0.1, 0.1, 0.1};
    if (robot->environment->getName() == "fullApartement")
      alphas = { 0.1, 0.1, 0.1, 0.1, 0.1};

    unsigned int totalIterations = 0;
    while (totalIterations < iterationsToSimulate) {
      // robot->reset();
      if (dynamic_cast<RobotAntelope*>(robot) != NULL) {
        Environment* env = robot->environment;
        Ground* gr = dynamic_cast<RobotAntelope*>(robot)->ground;
        gr->reset();
        env->reset();
        robot = new RobotAntelope(gr, env, poss[totalIterations], alphas[totalIterations]);
      }
      else if (dynamic_cast<RobotGoat*>(robot) != NULL) {
        Environment* env = robot->environment;
        env->reset();
        robot = new RobotGoat(env, poss[totalIterations], alphas[totalIterations]);
      }

      Json testArr = Json::array();

      unsigned int totalMinutesPassed = 0;
      while (totalMinutesPassed < minutesToSimulate) {
        robot->simulateOneSecond();

        Robot::PassedTime passedTime = robot->time_passed();
        totalMinutesPassed = passedTime.minutes + passedTime.hours * 60;
        if (totalMinutesPassed % 10 == 1 && passedTime.seconds == 0) {
          printf("%zu:%zu:%zu - %.2f", passedTime.hours, passedTime.minutes, passedTime.seconds, robot->percentage() * 100.0f);
          std::cout << "%" << std::endl;
        }

        testArr.push_back(Json::object({{"time", (float)totalMinutesPassed + (float)passedTime.seconds / 60.0f}, {"percentage", robot->percentage() * 100.0f}, {"x", robot->currentPosition().x}, {"y", robot->currentPosition().y}, {"alpha", robot->getAlpha()}}));
      }
      data.push_back(testArr);

      totalIterations++;
    }
  }
};