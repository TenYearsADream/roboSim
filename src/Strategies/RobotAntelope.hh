#pragma once

#include "../Robot.hh"
#include "Modules/GroundModule.hh"

#define FOURCONNECTED true

enum RoboStatus { rs_random = 0, rs_followCurrentPath, rs_followFinalPath, rs_stopped, rs_explore };
enum FollowCurrentPathStatus { fcps_start = 0, fcps_nearCurrentNode, fcps_movingToNextNode, fcps_finished };

class RobotAntelope : public Robot {
public:
  Ground* ground;
  GroundModule groundModule;
  std::vector<Vec> unknownPoints;
  int drawTargetGround = 0;
  std::vector<Polygon> polygons;
  std::vector<size_t> pathPointIndices;
  RoboStatus currentStatus = rs_explore;
  FollowCurrentPathStatus followCurrentPathStatus = fcps_start;
  int selectedPolygon = -1;
  bool renderPath = false;
  bool cout_time = false;

  RobotAntelope(Ground* _ground, Environment* _environment, Vec _position, float _alpha) : ground(_ground), groundModule(GroundModule(_ground)), Robot(_environment, _position, _alpha) {}

  std::string getName() {
    return "Antelope";
  }

  int blaIndex(int x, int y, int maxx = 5) {
    return maxx * y + x;
  }

  void handleKeys(unsigned char key, int x, int y) {
    if (key == 'p') {
      computePath();
    }

    if (key == 'o') {
      renderPath = !renderPath;
    }

    if (key == 'f') {
      if (currentStatus == rs_random) {
        currentStatus = rs_followCurrentPath;
        followCurrentPathStatus = fcps_start;
      }
      else
        currentStatus = rs_random;
    }

    // if (key == 't') {
    //   cout_time = !cout_time;
    // }
    if (key == 'g') {
      drawTargetGround = (drawTargetGround + 1) % 3;
    }
    if (key == 'a') {
      // std::cout << "Lineee: " << environment->lines[0] << std::endl;
      // environment->target_ground.markGroundTypeNearLineRadius(environment->lines[0], getRadius() * 1.1f, gt_wall);
    }
    if (key == 'c') {
      Vec selectedPos = GLUtils::getSelectedPos(x, y);
      // ground->fill(gt_wall);
      // for (int i = 1; i < 5; i++)
      //   for (int j = 1; j < 5; j++)
      //     ground->groundType(i, j) = gt_probably_reachable_seen;
      // ground->groundType(1, 1) = gt_visited;
      // DEBUG_VAR(selectedPos);
      // DEBUG_VAR(ground->coordToGlobalIndex(selectedPos));

      // Eigen::MatrixXi graphAccessible = Eigen::MatrixXi::Zero(25, 25);
      // for (unsigned int x = 0; x < 4; x++) {
      //   for (unsigned int y = 0; y < 4; y++) {
      //     if (ground->isReachable(x, y)) {
      //       if (ground->isReachable(x + 1, y))
      //         Ground::connectToPointsInGraph(graphAccessible, blaIndex(x, y), blaIndex(x + 1, y));
      //       if (ground->isReachable(x, y + 1))
      //         Ground::connectToPointsInGraph(graphAccessible, blaIndex(x, y), blaIndex(x, y + 1));
      //     }
      //   }
      // }

      // DEBUG_VAR(graphAccessible);
      // pathPointIndices = primMST(graphAccessible, 0);

      // Ground ground(5, 5, 1.0f);
      // for (int i = 0; i < 5; i++)
      //   for (int j = 0; j < 5; j++)
      //     if (i == 2 || j == 2)
      //       ground.groundType(i, j) = gt_probably_reachable_seen;
      //     else
      //       ground.groundType(i, j) = gt_visited;
      // // ground.groundType(0, 0) = gt_visited;
      // ground.getRepresentativesOfConnectedRegions();

      // computePath(ground->coordToGlobalIndex(selectedPos));
      pathPointIndices = ground->computePathOfUncleanedMSTs(ground->coordToGlobalIndex(currentPosition()), FOURCONNECTED);
      currentStatus = rs_followFinalPath;
      followCurrentPathStatus = fcps_start;
    }

    if (key == 'e') {
      Vec selectedPos = GLUtils::getSelectedPos(x, y);

      // ground->markVisitedInRadius(selectedPos, 0.1f);

      pathPointIndices = ground->shortestPath(ground->vecToVeci(currentPosition()), ground->vecToVeci(selectedPos), FOURCONNECTED);

      // pathPointIndices = ground->getFirstReachableUnknownPoint(ground->vecToVeci(currentPosition()), getRadius(), FOURCONNECTED, true);

      /*pathPointIndices = {183, 123, 63, 3, 2, 1, 0};
      ground->groundType(ground->globalIndexToVeci(4)) = gt_probably_reachable_seen;
      ground->groundType(ground->globalIndexToVeci(64)) = gt_probably_reachable_seen;*/
      ground->circumNavigateCornersIfPossible(pathPointIndices);
      DEBUG_VEC(pathPointIndices);
      // currentStatus = rs_followCurrentPath;
      // followCurrentPathStatus = fcps_start;
    }

    if (key == 'i') {
      std::cout << "Percentage: " << percentage() * 100.0f << "%" << std::endl;
      std::cout << "Pos: " << currentPosition() << std::endl;
      std::cout << "Alpha: " << getAlpha() << std::endl;
      Vec selectedPos = GLUtils::getSelectedPos(x, y);
      std::cout << "selectedPos: " << selectedPos << std::endl;
    }

    if (key == 'i') {
      showPolygonInfo();
    }
  }

  void draw_specific() {
    environment->draw();
    switch (drawTargetGround) {
      case 2:
        environment->ground.draw(); break;
      case 1:
        environment->target_ground.draw(); break;
      default:
        ground->draw(); break;
    }

    GLUtils::drawCrosses(Config::color_unknownPoints, unknownPoints, 0.06);

    for (size_t i = 0; i < polygons.size(); i++)
      if (selectedPolygon == i)
        GLUtils::drawPolygon(Config::color_polygon_selected, polygons[i]);
      else
        GLUtils::drawPolygon(Config::color_polygons, polygons[i]);

    if (renderPath && pathPointIndices.size()) {
      GLUtils::drawPath(Config::color_path, pathPointIndices, ground);
    }

    if (cout_time) {
      PassedTime passedTime = time_passed();
      std::cout << getMillisecondsSimulated() << "\t/\t" << passedTime.seconds << "\t/\t" << passedTime.minutes << "\t/\t" << passedTime.hours << std::endl;
    }
  }

  void moveForwardOneTenthSecond() {
    moveForward();
    ground->markVisitedInRadius(currentPosition(), getRadius());
  }

  void simulateOneTenthSecond_specific() { // Simulate one tenth of a second
    // if (milliseconds_simulated == 1000 * 60 * 30) {
    //   x = 0.5;
    //   y = 0.5;
    //   currentStatus = rs_followCurrentPath;
    // }
    // moveForwardOneTenthSecond();
    // return;

    /*if (currentStatus == rs_random) {
      checkFront(getRadiusSearch());
      if (!checkFront(getRadius() * 2.0)) {
        moveForwardOneTenthSecond();
      }
      else {
        for (int i = 0; i < 360; i++) {
          rotateDegree(2.0f);
          checkFront(getRadiusSearch());
        }
        ground->localMaximaFilter(0.1);//0.03
        ground->markGridPointsNearKnownPointsAsWalls(getRadius() * 1.1f);
        // computeLinesHough();
        // //mergeLines();
        // shortenLines();
        // connectEndpoints();
        // connectPoints();
        findPointsInPolygons();
        computePath();
        rotateDegree(rand() % 360);
      }
    }
    else if (currentStatus == rs_followCurrentPath) {
      followCurrentPath();
    }*/

    if (currentStatus == rs_explore) {
      if (followCurrentPathStatus == fcps_finished) {
        if (ground->unknownNeighbor(ground->vecToVeci(currentPosition())))
          exploreNeighborhood();

        pathPointIndices = ground->getFirstReachableUnknownPoint(ground->vecToVeci(currentPosition()), getRadius(), FOURCONNECTED, true);
        // DEBUG_VEC(pathPointIndices);
        if (!pathPointIndices.size()) {
          computePath();
          currentStatus = rs_followFinalPath;
        }
        followCurrentPathStatus = fcps_start;
      }
      else {
        followCurrentPath();
      }
    }
    else if (currentStatus == rs_followFinalPath) {
      if (followCurrentPathStatus == fcps_finished) {
        currentStatus = rs_stopped;
        setFinished();
      }
      else {
        followCurrentPath();
      }
    }
  }

  bool checkFront(float _radius_search) {
    Vec intersection;
    bool intersected;
    float distFront = frontDistance(intersected, intersection);
    if (intersected)
      ground->knownPoints.push_back(intersection);

    Vec direction = ((intersected ? intersection : currentPosition().withAddedPolarCoordinates(getRadiusSearch(), getAlpha())) - currentPosition());
    for (int i = 0; i < 20; i++) {
      Vec p = currentPosition() + direction / 20.0f * (float)i;
      ground->markGroundTypeRadius(p, ground->tickSpacing / 2.0f, gt_probably_reachable_seen, {gt_unknown, gt_probably_reachable_polygon});
    }

    return intersected && distFront < _radius_search;
  }

  void exploreNeighborhood() {
    int step = 2;
    for (int i = 0; i < 360; i += step) {
      rotateDegree(step);
      checkFront(getRadiusSearch());
    }
    ground->localMaximaFilter(0.05);//0.03
    ground->markGridPointsNearKnownPointsAsWalls(getRadius() * 1.3f);
  }

  Vec getPosFromID(unsigned int id) { int yy = id / ground->maxx; int xx = id - yy * ground->maxx; return Vec(ground->indexXToCoord(xx), ground->indexYToCoord(yy)); }
  unsigned int getIDFromCoord(unsigned int _x, unsigned int _y) { return ground->maxx * _y + _x; }

  void followCurrentPath() {
    if (!pathPointIndices.size()) {
      followCurrentPathStatus = fcps_finished;
      return;
    }

    if (followCurrentPathStatus == fcps_movingToNextNode) {
      Vec target = getPosFromID(pathPointIndices.back());
      if (dist(target) > 0.05) {
        setAlpha(-M_PI / 2.0f - (currentPosition() - target).angleToXAxis());
        moveForwardOneTenthSecond();
      }
      else {
        pathPointIndices.resize(pathPointIndices.size() - 1);
        if (!pathPointIndices.size()) {
          exploreNeighborhood();
          followCurrentPathStatus = fcps_finished;
        }
        else {
          followCurrentPathStatus = fcps_nearCurrentNode;
        }
      }
    }
    else if (followCurrentPathStatus == fcps_start || followCurrentPathStatus == fcps_nearCurrentNode) {
      // alpha = -(atan((y-getPosFromID(pathPointIndices.back()).y)/(x-getPosFromID(pathPointIndices.back()).x)) + M_PI/4.0);
      followCurrentPathStatus = fcps_movingToNextNode;
    }
  }

  void computePath(long long startingPos = -1) {
    if (startingPos < 0)
      startingPos = ground->coordToGlobalIndex(currentPosition());

    /*Eigen::MatrixXi graphAccessible = Eigen::MatrixXi::Zero(ground->maxx * ground->maxy, ground->maxx * ground->maxy);
    Eigen::MatrixXi graphUncleaned  = Eigen::MatrixXi::Zero(ground->maxx * ground->maxy, ground->maxx * ground->maxy);
    for (unsigned int x = 0; x < ground->maxx - 1; x++) {
      for (unsigned int y = 0; y < ground->maxy - 1; y++) {
        if (ground->isReachable(x, y)) {
          if (ground->isReachable(x + 1, y))
            connectToPointsInGraph(graphAccessible, getIDFromCoord(x, y), getIDFromCoord(x + 1, y));
          if (ground->isReachable(x, y + 1))
            connectToPointsInGraph(graphAccessible, getIDFromCoord(x, y), getIDFromCoord(x, y + 1));
        }

        if (ground->isReachableButUnvisited(x, y)) {
          if (ground->isReachableButUnvisited(x + 1, y))
            connectToPointsInGraph(graphUncleaned, getIDFromCoord(x, y), getIDFromCoord(x + 1, y));
          if (ground->isReachableButUnvisited(x, y + 1))
            connectToPointsInGraph(graphUncleaned, getIDFromCoord(x, y), getIDFromCoord(x, y + 1));
        }
      }
    }
    for (unsigned int i = 0; i < graphAccessible.cols(); i++) {
      for (unsigned int j = 0; j < graphAccessible.cols(); j++) {
        if (graphAccessible(i, j) > 0 && graphUncleaned(i, j) == 0) {
          // Compute minimal path in graphAccessible and set weight in graphUncleaned to path length
        }
      }
    }
    pathPointIndices = primMST(graphAccessible, startingPos);*/
    // pathPointIndices = ground->computeMSTPathUncleaned(startingPos);
    pathPointIndices = ground->computePathOfUncleanedMSTs(startingPos, FOURCONNECTED);
  }

  void connectPoints() {
    polygons.clear();
    Polygon pol;
    Vec p = ground->knownPoints[0];
    pol.push_back(p);
    polygons.push_back(pol);

    std::vector<size_t> ids;
    ids.push_back(0);

    bool end = false;
    while (!end) {
      long minIndex = -1;
      float minDist = 1000000;
      for (size_t j = 0; j < ground->knownPoints.size(); j++) {
        float dist = p.dist(ground->knownPoints[j]);
        bool alreadyInList = (find(ids.begin(), ids.end(), j) != ids.end());
        if (dist < minDist && dist > 0.001 && !alreadyInList) {
          minIndex = j;
          minDist = dist;
        }
      }

      if (minDist < 0.3) {
        ids.push_back(minIndex);

        p = ground->knownPoints[minIndex];
        polygons.back().push_back(p);
      }
      else {
        for (size_t j = 0; j < ground->knownPoints.size(); j++) {
          bool alreadyInList = (find(ids.begin(), ids.end(), j) != ids.end());
          if (!alreadyInList) {
            Polygon pol;
            p = ground->knownPoints[j];
            ids.push_back(j);
            pol.push_back(p);
            polygons.push_back(pol);
            break;
          }
        }
      }

      size_t pointsInPolygons = 0;
      for (size_t j = 0; j < polygons.size(); j++)
        pointsInPolygons += polygons[j].size();

      end = pointsInPolygons == ground->knownPoints.size();
    }
    // std::cout << "Polygon sizes: " << std::endl;
    // for (size_t i = 0; i < polygons.size(); i++)
    //   std::cout << polygons[i].size() << ", ";
    // std::cout << std::endl;
    // std::cout << "Polygons: " << polygons.size() << std::endl;
  }

  void findPointsInPolygons() {
    ground->resetProbableGroundToUnknown();
    ground->markGroundTypePolygons(polygons, 0.0f);
  }

  void showPolygonInfo() {
    if (selectedPolygon >= 0 && selectedPolygon < polygons.size()) {
      for (size_t i = 0; i < polygons[selectedPolygon].size(); i++) {
        std::cout << "(" << polygons[selectedPolygon][i].x << "|" << polygons[selectedPolygon][i].y << ")" << std::endl;
      }

      std::vector<OpenMesh::Vec3f> points;
      for (size_t i = 0; i < polygons[selectedPolygon].size(); i++) {
        points.push_back(OpenMesh::Vec3f(polygons[selectedPolygon][i].x, polygons[selectedPolygon][i].y, 0));
      }
      ACG::Triangulator triangulator(points);
      std::vector<int> indices = triangulator.indices();
      size_t numTriangles = indices.size() / 3;

      bool wasInside1 = false;
      bool wasInside2 = false;
      bool wasInside3 = false;
      bool wasInside4 = false;

      for (size_t j = 0; j < numTriangles; j++) {
        std::cout << indices[j * 3] << "/" << indices[j * 3 + 1] << "/" << indices[j * 3 + 2] << "  " << polygons[selectedPolygon][indices[j * 3]] << "/" << polygons[selectedPolygon][indices[j * 3 + 1]] << "/" << polygons[selectedPolygon][indices[j * 3 + 2]] << std::endl;
        if (MathUtils::PointInTriangle(Vec(0.0f, 4.0f), polygons[selectedPolygon][indices[j * 3]], polygons[selectedPolygon][indices[j * 3 + 1]], polygons[selectedPolygon][indices[j * 3 + 2]]))
        {std::cout << "*1*"; wasInside1 = true;}
        if (MathUtils::PointInTriangle(Vec(0.0f, 5.0f), polygons[selectedPolygon][indices[j * 3]], polygons[selectedPolygon][indices[j * 3 + 1]], polygons[selectedPolygon][indices[j * 3 + 2]]))
        {std::cout << "*2*"; wasInside2 = true;}
        if (MathUtils::PointInTriangle(Vec(1.4f, 1.0f), polygons[selectedPolygon][indices[j * 3]], polygons[selectedPolygon][indices[j * 3 + 1]], polygons[selectedPolygon][indices[j * 3 + 2]]))
        {std::cout << "*3*"; wasInside3 = true;}
        if (MathUtils::PointInTriangle(Vec(0.2f, 0.2f), polygons[selectedPolygon][indices[j * 3]], polygons[selectedPolygon][indices[j * 3 + 1]], polygons[selectedPolygon][indices[j * 3 + 2]]))
        {std::cout << "*4*"; wasInside4 = true;}
      }
      std::cout << wasInside1 << "/" << wasInside2 << "/" << wasInside3 << "/" << wasInside4 << std::endl;
    }
  }
};