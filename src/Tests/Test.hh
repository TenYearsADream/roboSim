#pragma once

#include "../../extern/json/json.hh"
#include "../Robot.hh"

using Json = nlohmann::json;

class Test {
public:
  Robot* robot;
  Json data;

  Test(Robot* _robot) : robot(_robot) {}

  virtual std::string getName_specific() const = 0;
  std::string getName() const {
    return getName_specific() + "__" + robot->environment->getName();
  }
  virtual void runTest() {
    std::cout << "Running test " << getName_specific() << " in " << robot->environment->getName() << " with " << robot->getName() << std::endl;
    robot->environment->reset();
    runTest_specific();
  };
  virtual void runTest_specific() = 0;
  virtual Json getData() const {
    return data;
  }
};