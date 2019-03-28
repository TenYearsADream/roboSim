#pragma once

class RobotChipmunk : public Robot {
private:
  std::vector<Line> knownLines;
  std::vector<Vec> pointsInPolygon;
  std::vector<Vec> pointsInPolygon2;

public:
  RobotChipmunk(Environment* _environment, Vec _position, float _alpha) : Robot(_environment, _position, _alpha) {}

  void handleKeys(unsigned char key, int x, int y) {
    // if (key == 'f') {
    //   findPointsInPolygons();
    // }

    if (key == 'g') {
      // findPolygon(true);
    }

    if (key == 'c') {
      // connectPoints();
    }

    if (key == 'v') {
      // mergeLines();
    }
  }

  std::string getName() {
    return "Chipmunk";
  }

  void simulateOneTenthSecond_specific() {
    if (milliseconds_simulated / 1000 % (5) == 0)
      rotateDegree(rand() % 360);
    else
      moveForward();
  }

  void draw_specific() {
    environment->draw();
    environment->ground.draw();

    GLUtils::drawLines(Config::color_lines_known, knownLines);
  }

  // void computeLinesHough() {
  //   Hough::Hough hough;
  //   std::vector<std::pair<float, float> > points;
  //   for (size_t i = 0; i < ground->knownPoints.size(); i++)
  //     points.push_back(ground->knownPoints[i].getPair());
  //   hough.Transform(points, 100, 400, 600);

  //   knownLines.clear();
  //   std::vector< std::pair< std::pair<int, int>, std::pair<int, int> > > lines = hough.GetLines(20);
  //   for (size_t i = 0; i < lines.size(); i++)
  //     knownLines.push_back(Line(Vec(lines[i].first.first / 100.0f, lines[i].first.second / 100.0f), Vec(lines[i].second.first / 100.0f, lines[i].second.second / 100.0f)));
  // }

  // void shortenLines() {
  //   for (size_t i = 0; i < knownLines.size(); i++) {
  //     // std::cout << "Line " << i << std::endl;
  //     /*int mode = 0;
  //     bool end = false;
  //     float origEndx = knownLines[i].x2;
  //     float origEndy = knownLines[i].y2;
  //     float lastS = 0.0f;
  //     for (float s = 0.0f; s < 1.0f && !end; s+=0.01f) {
  //       // std::cout << "s=" << s << std::endl;
  //       Vec p = Vec(knownLines[i].x1 + s*(knownLines[i].x2-knownLines[i].x1), knownLines[i].y1 + s*(knownLines[i].y2-knownLines[i].y1));
  //       if (mode == 0) {
  //         for (size_t j = 0; j < ground->knownPoints.size(); j++) {
  //           if (p.dist(ground->knownPoints[j]) < 0.07) {
  //             knownLines[i].x1 = p.x;
  //             knownLines[i].y1 = p.y;
  //             mode = 1;
  //             lastS = s;
  //             break;
  //           }
  //         }
  //       }
  //       else if (mode == 1) {
  //         bool found = false;
  //         for (size_t j = 0; j < ground->knownPoints.size(); j++) {
  //           if (p.dist(ground->knownPoints[j]) < 0.2) {
  //             found = true;
  //             break;
  //           }
  //         }
  //         if (!found) {
  //           knownLines[i].x2 = p.x;
  //           knownLines[i].y2 = p.y;
  //           mode = 2;
  //           lastS = s;
  //         }
  //       }
  //       else if (mode == 2) {
  //         for (size_t j = 0; j < ground->knownPoints.size(); j++) {
  //           if (p.dist(ground->knownPoints[j]) < 0.07 && s-lastS > 0.00) {
  //             std::cout << "Added one line. Line: " << i << "  " << p.x << "/" << p.x << " " << origEndx << "/" << origEndy << "   " << "p=" << p.x << "/" << p.y << std::endl;
  //             // knownLines.push_back(Line(p, Vec(origEndx, origEndy)));
  //             end = true;
  //             break;
  //           }
  //         }
  //       }
  //     }
  //     if (mode == 2 && !end) {
  //       // std::cout << "Still mode 0: Line " << i << std::endl;
  //       knownLines.erase(knownLines.begin()+i);
  //       i--;
  //     }*/

  //     float lastS = 0.0f;
  //     bool end = false;
  //     bool deleteUntil = false;
  //     Vec pUntil(knownLines[i].x1, knownLines[i].x2);
  //     Vec lastp(knownLines[i].x1, knownLines[i].y1);
  //     for (float s = 0.0f; s < 1.0f && !end; s += 0.01f) {
  //       bool found = false;
  //       Vec p = Vec(knownLines[i].x1 + s * (knownLines[i].x2 - knownLines[i].x1), knownLines[i].y1 + s * (knownLines[i].y2 - knownLines[i].y1));
  //       for (size_t j = 0; j < ground->knownPoints.size(); j++) {
  //         if (p.dist(ground->knownPoints[j]) < 0.01) {
  //           found = true;
  //           if (lastS < 0.001f) {
  //             deleteUntil = true;
  //             pUntil = p;
  //           }
  //           if (s - lastS > 0.15f) {
  //             knownLines.push_back(Line(p, Vec(knownLines[i].x2, knownLines[i].y2)));
  //             knownLines[i].x2 = lastp.x;
  //             knownLines[i].y2 = lastp.y;
  //             end = true;
  //           }
  //           lastS = s;
  //           lastp = p;
  //           break;
  //         }
  //       }
  //     }
  //     if (!end) {
  //       knownLines[i].x2 = lastp.x;
  //       knownLines[i].y2 = lastp.y;
  //     }
  //     /*if (deleteUntil) {
  //       knownLines[i].x1 = pUntil.x;
  //       knownLines[i].y1 = pUntil.y;
  //     }*/
  //   }
  //   for (long long i = knownLines.size() - 1; i >= 0; i--) {
  //     if (knownLines[i].getStart().dist(knownLines[i].getEnd()) < 0.02)
  //       knownLines.erase(knownLines.begin() + i);
  //   }
  // }

  // void connectEndpoints() {
  //   size_t maxSize = knownLines.size();
  //   for (size_t i = 0; i < maxSize; i++) {
  //     bool foundStartCompanion = false;
  //     bool foundEndCompanion = false;
  //     Vec start1 = knownLines[i].getStart();
  //     Vec end1 = knownLines[i].getEnd();

  //     for (size_t j = 0; j < maxSize; j++) {
  //       if (i == j)
  //         continue;

  //       float thresh1 = 0.5;
  //       float thresh2 = 0.2;

  //       Vec start2 = knownLines[j].getStart();
  //       Vec end2 = knownLines[j].getEnd();

  //       if (start1.dist(start2) < thresh1) {
  //         knownLines[j].setStart(start1);
  //         foundStartCompanion = true;
  //       }

  //       else if (start1.dist(end2) < thresh1) {
  //         knownLines[j].setEnd(start1);
  //         foundStartCompanion = true;
  //       }

  //       if (end1.dist(start2) < thresh1) {
  //         knownLines[j].setStart(end1);
  //         foundEndCompanion = true;
  //       }

  //       else if (end1.dist(end2) < thresh1) {
  //         knownLines[j].setEnd(end1);
  //         foundEndCompanion = true;
  //       }


  //       if (start1.dist(start2) < thresh2) {
  //         knownLines.push_back(Line(start1, start2));
  //         foundStartCompanion = true;
  //       }

  //       else if (start1.dist(end2) < thresh2) {
  //         knownLines.push_back(Line(start1, end2));
  //         foundStartCompanion = true;
  //       }

  //       if (end1.dist(start2) < thresh2) {
  //         knownLines.push_back(Line(end1, start2));
  //         foundEndCompanion = true;
  //       }

  //       else if (end1.dist(end2) < thresh2) {
  //         knownLines.push_back(Line(end1, end2));
  //         foundEndCompanion = true;
  //       }
  //     }
  //     if (!foundStartCompanion) {
  //       ground->knownPoints.push_back(start1);
  //     }
  //     if (!foundEndCompanion) {
  //       ground->knownPoints.push_back(end1);
  //     }
  //   }
  // }

  // void mergeLines() {
  //   size_t maxSize = knownLines.size();
  //   for (size_t i = 0; i < maxSize; i++) {
  //     for (size_t j = 0; j < maxSize; j++) {
  //       if (i == j)
  //         continue;

  //       // Parallel
  //       if (MathUtils::almostEqual(knownLines[i].x2 - knownLines[i].x1, knownLines[j].x2 - knownLines[j].x1, 0.05)
  //           && MathUtils::almostEqual(knownLines[i].y2 - knownLines[i].y1, knownLines[j].y2 - knownLines[j].y1, 0.05)) {
  //         Eigen::Vector2f dir1(knownLines[i].x2 - knownLines[i].x1, knownLines[i].y2 - knownLines[i].y1);
  //         Eigen::Vector2f dir2(knownLines[j].x2 - knownLines[j].x1, knownLines[j].y2 - knownLines[j].y1);
  //         Eigen::Vector2f dir1n = dir1.normalized();
  //         Eigen::Vector2f dir2n = dir2.normalized();

  //         Eigen::Vector2f base1(knownLines[i].x1, knownLines[i].y1);
  //         Eigen::Vector2f base2(knownLines[j].x1, knownLines[j].y1);

  //         Eigen::Vector2f end1(knownLines[i].x2, knownLines[i].y2);
  //         Eigen::Vector2f end2(knownLines[j].x2, knownLines[j].y2);

  //         float length1 = dir1.norm();
  //         float length2 = dir2.norm();

  //         Eigen::Vector2f n;
  //         n << dir1(1), dir1(0);
  //         n.normalize();
  //         float d = fabs(n.dot(base2 - base1));
  //         // std::cout << d << std::endl;

  //         if (d < 0.05) {
  //           float distStart1Start2 = dir1n.dot(base2 - base1);
  //           float distStart1End2 = dir1n.dot(end2 - base1);
  //           float distEnd1Start2 = dir1n.dot(base2 - end1);
  //           float distEnd1End2 = dir1n.dot(end2 - end1);

  //           if (distStart1Start2 > 0 && distEnd1Start2 < 0 && distEnd1End2 > 0) {
  //             knownLines[i].x2 = knownLines[j].x2;
  //             knownLines[i].y2 = knownLines[j].y2;

  //             knownLines[j].x1 = knownLines[j].x2;
  //             knownLines[j].y1 = knownLines[j].y2;
  //           }
  //           else if (distStart1Start2 < 0 && distEnd1Start2 > 0 && distEnd1End2 < 0) {
  //             knownLines[i].x1 = knownLines[j].x1;
  //             knownLines[i].y1 = knownLines[j].y1;

  //             knownLines[j].x1 = knownLines[j].x2;
  //             knownLines[j].y1 = knownLines[j].y2;
  //           }
  //           else if (distStart1Start2 > 0 && distEnd1Start2 < 0 && distEnd1End2 < 0) {
  //             knownLines[j].x1 = knownLines[j].x2;
  //             knownLines[j].y1 = knownLines[j].y2;
  //           }
  //           else if (distStart1Start2 < 0 && distEnd1Start2 < 0 && distEnd1End2 > 0) {
  //             knownLines[j].x1 = knownLines[j].x2;
  //             knownLines[j].y1 = knownLines[j].y2;
  //           }
  //         }
  //       }
  //     }
  //   }
  // }

  // std::vector<Line> findNeighboringLines(Vec _p) {
  //   std::vector<Line> lines;
  //   float thres = 0.05;
  //   float minlengthThres = 0.1;
  //   for (size_t i = 0; i < knownLines.size(); i++) {
  //     if (_p.dist(knownLines[i].getStart()) < thres && knownLines[i].length() > minlengthThres) {
  //       lines.push_back(knownLines[i]);
  //     }
  //     else if (_p.dist(knownLines[i].getEnd()) < thres && knownLines[i].length() > minlengthThres) {
  //       lines.push_back(Line(knownLines[i].getEnd(), knownLines[i].getStart()));
  //     }
  //   }
  //   return lines;
  // }

  // void findPolygon(bool _randomized) {
  //   /*if (knownLines.size() == 0)
  //     return;

  //   std::vector<Polygon> polygons;
  //   Polygon initialPolygon;
  //   initialPolygon.push_back(knownLines[_randomized ? rand() % knownLines.size() : 0].getStart());
  //   polygons.push_back(initialPolygon);

  //   Polygon finalPolygon;

  //   bool allPolygonsClosed = false;
  //   while (!allPolygonsClosed) {
  //     float thres = 0.01;
  //     std::vector<Polygon > new_polygons;
  //     for (size_t i = 0; i < polygons.size(); i++) {
  //       size_t current_size = polygons[i].size();
  //       if (!polygons[i].closed) {
  //         std::vector<Line> lines = findNeighboringLines(polygons[i][current_size-1]);
  //         for (size_t j = 0; j < lines.size(); j++) {
  //           Polygon polygon = polygons[i];
  //           polygon.push_back(lines[i].getEnd());
  //           if (polygon.hasCycle())
  //             polygon.closed = true;
  //           new_polygons.push_back(polygon);
  //         }
  //       }
  //     }

  //     if (new_polygons.size() == 0) {
  //       allPolygonsClosed = true;
  //       break;
  //     }

  //     std::cout << "new Polygons: " << std::endl;
  //     for (size_t i = 0; i < new_polygons.size(); i++) {
  //       for (size_t j = 0; j < new_polygons[i].size(); j++) {
  //         std::cout << new_polygons[i][j] << "-";
  //       }
  //       if (new_polygons[i].closed)
  //         std::cout << "closed";
  //       std::cout << std::endl;
  //     }

  //     allPolygonsClosed = true;
  //     for (size_t i = 0; i < new_polygons.size(); i++) {
  //       if (!new_polygons[i].closed) {
  //         // foundClosedPolygon = true;
  //         allPolygonsClosed = false;
  //         break;
  //       }
  //     }

  //     polygons = new_polygons;
  //   }
  //   // std::cout << "Found polygon: " << foundClosedPolygon << std::endl;
  //   polygon = finalPolygon;*/
  // }
};