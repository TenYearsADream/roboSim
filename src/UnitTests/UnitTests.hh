#pragma once

#include "gtest/gtest.h"

#include "../Geometry/Environment.hh"

TEST(MathTest, intersectionWithMovement) {
  Ground ground(10, 10, 0.2f);
  Environment environment(Ground(10, 10, 0.2f));

  environment.lines.push_back(Line(Vec(0, 5), Vec(4, 5)));

  float moveableDistance;
  bool collision = environment.intersectionWithMovement(Vec(2, 0), 8, 0, 1, moveableDistance);
  EXPECT_EQ(moveableDistance, 4);
  EXPECT_EQ(collision, true);
  std::cout << "==========" << std::endl;

  collision = environment.intersectionWithMovement(Vec(5.1, 0), 8, 0, 1, moveableDistance);
  EXPECT_EQ(moveableDistance, 8);
  EXPECT_EQ(collision, false);
  std::cout << "==========" << std::endl;

  collision = environment.intersectionWithMovement(Vec(4.5, 0), 8, 0, 1, moveableDistance);
  // EXPECT_EQ(moveableDistance, 8);
  EXPECT_EQ(collision, true);
  std::cout << "==========" << std::endl;

  Vec pos = Vec(0.95, 0.8);
  float alpha = -0.2;
  environment.lines = { Line(Vec(1, 1), Vec(2, 1)), Line(Vec(1, 1), Vec(1, 2)) };
  collision = environment.intersectionWithMovement(pos, 8, alpha, 0.15, moveableDistance);
  EXPECT_EQ(collision, true);
  EXPECT_LT(moveableDistance, 8);
  pos = pos.withAddedPolarCoordinates(moveableDistance, alpha);

  collision = environment.intersectionWithMovement(pos, 8, alpha, 0.15, moveableDistance);
  EXPECT_EQ(collision, true);
  EXPECT_LT(moveableDistance, 8);
  pos = pos.withAddedPolarCoordinates(moveableDistance, alpha);

  collision = environment.intersectionWithMovement(pos, 8, alpha, 0.15, moveableDistance);
  EXPECT_EQ(collision, true);
  EXPECT_LT(moveableDistance, 8);
  pos = pos.withAddedPolarCoordinates(moveableDistance, alpha);

}

TEST(MathTest, intersectionWithCircle) {
  Line line(Vec(0, 1), Vec(4, 1));

  std::vector<Vec> intersections = line.intersectWithCircle(Vec(2, 1), 1);
  EXPECT_EQ(intersections.size(), 2);
  std::cout << "==========" << std::endl;

  intersections = line.intersectWithCircle(Vec(4.5, 1), 1);
  EXPECT_EQ(intersections.size(), 1);
}