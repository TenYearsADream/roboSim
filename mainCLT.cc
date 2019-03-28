#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <time.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>

#include <curl/curl.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "extern/hough/hough.hh"
#include "extern/shortestPaths/shortestPaths.hh"
#include "extern/dijkstra/dijkstra.hh"
#include "extern/json/json.hh"

#ifdef BUILD_WITH_GTEST
#include "gtest/gtest.h"
#include "src/UnitTests/UnitTests.hh"
#endif

#include "src/Robot.hh"
#include "src/Strategies/RobotAntelope.hh"
#include "src/Strategies/RobotGoat.hh"
#include "src/Network/Network.hh"
#include "src/Tests/SimpleTest.hh"
#include "src/Utils/Utils.hh"

using Json = nlohmann::json;

//Screen dimension constants
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

const int GROUND_WIDTH = 12;
const int GROUND_HEIGHT = 12;

void runTests(int _minutesToSimulate = 10, bool _upload = false) {
  Ground ground(GROUND_WIDTH, GROUND_HEIGHT, 0.2f);
  Ground ground2(GROUND_WIDTH, GROUND_HEIGHT, 0.2f);
  Environment environment(Ground(GROUND_WIDTH, GROUND_HEIGHT, 0.1f));
  Environment environment2(Ground(GROUND_WIDTH, GROUND_HEIGHT, 0.1f));

  Environment environment_full(Ground(GROUND_WIDTH, GROUND_HEIGHT, 0.1f));
  Environment environment2_full(Ground(GROUND_WIDTH, GROUND_HEIGHT, 0.1f));

  std::vector<Robot*> robots = {
    new RobotAntelope(&ground, &environment, Vec(0.5, 0.5), 0.1), new RobotGoat(&environment2, Vec(0.5, 0.5), 0.1),
    new RobotAntelope(&ground2, &environment_full, Vec(0.5, 0.5), 0.1), new RobotGoat(&environment2_full, Vec(0.5, 0.5), 0.1)
  };

  std::vector<Polygon> roomPolygons = { Polygon::generateRoom(), Polygon::generatePillar() };
  robots[0]->environment->initTargetWithPolygons(roomPolygons, robots[0]->getRadius(), "roomAndHall");
  robots[1]->environment->initTargetWithPolygons(roomPolygons, robots[1]->getRadius(), "roomAndHall");

  roomPolygons = { Polygon::generateRoomExtended(), Polygon::generatePillar() };
  robots[2]->environment->initTargetWithPolygons(roomPolygons, robots[2]->getRadius(), "fullApartement");
  robots[3]->environment->initTargetWithPolygons(roomPolygons, robots[3]->getRadius(), "fullApartement");

  std::vector<Test*> tests;
  tests.push_back(new SimpleTest(robots[0], _minutesToSimulate, 5));
  tests.push_back(new SimpleTest(robots[1], _minutesToSimulate, 1));
  tests.push_back(new SimpleTest(robots[2], _minutesToSimulate, 5));
  tests.push_back(new SimpleTest(robots[3], _minutesToSimulate, 1));

  for (size_t i = 0; i < tests.size(); i++) {
    tests[i]->runTest();
    if (_upload) {
      std::cout << "Starting upload" << std::endl;
      Network::upload(tests[i]->robot, *tests[i]);
    }
  }
}

int runUnitTests(int argc, char* args[]) {
#ifdef BUILD_WITH_GTEST
  ::testing::InitGoogleTest(&argc, args);
  return RUN_ALL_TESTS();
#else
  return 1;
#endif
}

int main(int argc, char* args[]) {
  // Ground ground(0.4, 0.4, 0.1f);
  // ground.groundType(0, 0) = gt_probably_reachable_seen;
  // ground.groundType(1, 0) = gt_probably_reachable_seen;
  // ground.groundType(2, 0) = gt_probably_reachable_seen;
  // ground.groundType(3, 1) = gt_probably_reachable_seen;
  // ground.groundType(0, 2) = gt_probably_reachable_seen;

  // ground.shortestPath(Veci(0, 0), Veci(3, 1), false);

  // // -> Unittest

  srand(time(NULL));

  int minutes = 10;
  int index;
  int c;
  bool upload = false;

  int mode = 0;

  opterr = 0;
  while ((c = getopt(argc, args, "um:t")) != -1)
    switch (c) {
      case 't':
        mode = 1;
        break;
      case 'u':
        upload = true;
        break;
      case 'm':
        minutes = atoi(optarg);
        break;
      case '?':
        if (optopt == 'c')
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        else if (isprint(optopt))
          fprintf(stderr, "Unknown option `-%c'.\n", optopt);
        else
          fprintf(stderr,
                  "Unknown option character `\\x%x'.\n",
                  optopt);
        return 1;
      default:
        abort();
    }

  for (index = optind; index < argc; index++)
    printf("Non-option argument %s\n", args[index]);

  if (mode == 0) {
    runTests(minutes, upload);
  }
  else if (mode == 1) {
    return runUnitTests(argc, args);
  }
  return 0;
}

