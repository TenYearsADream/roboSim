#pragma once

#include <Eigen/Dense>
#include "../Visualization/Drawable.hh"
#include "../Utils/MathUtils.hh"
#include "../Geometry/Line.hh"
#include "../Geometry/Polygon.hh"
#include "../../extern/OpenMesh/Triangulator.hh"
#include "../Defines.hh"
#include "../../extern/shortestPaths/shortestPaths.hh"
#include <unordered_set>
#include <algorithm>

enum GroundType {gt_unknown = 0, gt_visited, gt_probably_reachable_seen, gt_probably_reachable_polygon, gt_probably_unreachable, gt_wall};

class Ground : public Eigen::MatrixXi, public Drawable {
public:
  float tickSpacing;
  unsigned int groundwidth, groundheight, maxx, maxy;
  std::vector<Vec> knownPoints;

  Ground() {}
  Ground(const Ground& _otherground) : Ground(_otherground.groundwidth, _otherground.groundheight, _otherground.tickSpacing) {}
  Ground(float _groundwidth, float _groundheight, float _tickSpacing) : groundwidth(_groundwidth), groundheight(_groundheight), tickSpacing(_tickSpacing), maxx(ceil(_groundwidth / tickSpacing)), maxy(ceil(_groundheight / tickSpacing)) {
    resize(maxx, maxy);
    fill(0);
  }

  void reset() {
    fill(0);
    knownPoints.clear();
  }

  int& groundType(unsigned int _x, unsigned int _y) {
    return (*this)(_x, _y);
  }
  int groundType(unsigned int _x, unsigned int _y) const {
    return (*this)(_x, _y);
  }
  int& groundType(const Veci& _veci) {
    return (*this)(_veci.x, _veci.y);
  }
  int groundType(const Veci& _veci) const {
    return (*this)(_veci.x, _veci.y);
  }

  void draw();

  float compareToTarget(Ground* _target_ground) const {
    int sumTargetVisited = 0;
    int sumVisitedAsInTarget = 0;
    for (unsigned int x = 0; x < maxx; x++) {
      for (unsigned int y = 0; y < maxy; y++) {
        assert(validIndexX(x) && validIndexY(y) && _target_ground->validIndexX(x) && _target_ground->validIndexY(y));
        if (_target_ground->groundType(x, y) == gt_visited)
          sumTargetVisited++;
        if (groundType(x, y) == gt_visited && groundType(x, y) == _target_ground->groundType(x, y))
          sumVisitedAsInTarget++;
      }
    }
    return (float)sumVisitedAsInTarget / (float)sumTargetVisited;
  }

  bool isReachable(unsigned int _x, unsigned int _y) {
    return MathUtils::in(groundType(_x, _y), {gt_probably_reachable_seen, gt_probably_reachable_polygon, gt_visited});
  }
  bool isReachableButUnvisited(unsigned int _x, unsigned int _y) {
    return MathUtils::in(groundType(_x, _y), {gt_probably_reachable_seen, gt_probably_reachable_polygon});
  }
  bool isReachable(Veci _veci) {
    return isReachable(_veci.x, _veci.y);
  }
  bool isReachableButUnvisited(Veci _veci) {
    return isReachableButUnvisited(_veci.x, _veci.y);
  }

  bool isAllGroundtypeInRadius(const Vec& _pos, float _radius, std::vector<int> _groundtypeList = std::vector<int>()) {
    unsigned int xOnGround = coordXToIndex(_pos.x);
    unsigned int yOnGround = coordYToIndex(_pos.y);
    bool stop = false;
    int indexradius = 1;
    bool foundTypeNotInList = false;
    while (!stop) {
      for (int i = -indexradius; i < indexradius; i++) {
        for (int j = -indexradius; j < indexradius; j++) {
          stop = true;
          if (validIndexX(xOnGround + i) && validIndexY(yOnGround + j)) {
            if (indicesToCoords(xOnGround + i, yOnGround + j).dist(_pos) <= _radius) {
              stop = false;
              if (!MathUtils::in(groundType(xOnGround + i, yOnGround + j), _groundtypeList)) {
                foundTypeNotInList = true;
                stop = true; // No need to search farther
              }
            }
          }
        }
      }
      indexradius++;
    }
    return !foundTypeNotInList;
  }
  bool isAllReachableInRadius(const Vec& _pos, float _radius) {
    return isAllGroundtypeInRadius(_pos, _radius, {gt_probably_reachable_seen, gt_probably_reachable_polygon, gt_visited});
  }

  Veci findNearestReachablePoint(const Vec& _pos) {
    unsigned int xOnGround = coordXToIndex(_pos.x);
    unsigned int yOnGround = coordYToIndex(_pos.y);
    int indexradius = 1;
    while (indexradius < 10) { // Random value to avoid endless loop
      for (int i = -indexradius; i < indexradius; i++) {
        for (int j = -indexradius; j < indexradius; j++) {
          if (validIndexX(xOnGround + i) && validIndexY(yOnGround + j)) {
            if (isReachable(xOnGround + i, yOnGround + j))
              return Veci(xOnGround + i, yOnGround + j);
          }
        }
      }
      indexradius++;
    }
    throw 12345;
  }

  unsigned int coordXToIndex(float x) const { return lrint(x / tickSpacing); } // lrint = round to nearest neighbor
  unsigned int coordYToIndex(float y) const { return lrint(y / tickSpacing); }
  Veci vecToVeci(Vec _pos) const { return Veci(coordXToIndex(_pos.x), coordYToIndex(_pos.y)); }
  Vec veciToVec(Veci _veci) const { return Vec(indexXToCoord(_veci.x), indexYToCoord(_veci.y)); }
  float indexXToCoord(float x) const { return x * tickSpacing; }
  float indexYToCoord(float y) const { return y * tickSpacing; }
  Vec indicesToCoords(unsigned int i, unsigned int j) const { return Vec(indexXToCoord(i), indexYToCoord(j)); }
  bool validIndexX(int _index) const { return _index >= 0 && _index < cols(); }
  bool validIndexY(int _index) const { return _index >= 0 && _index < rows(); }
  bool validVeci(Veci _veci) const { return validIndexX(_veci.x) && validIndexY(_veci.y); }
  Vec globalIndexToCoord(size_t _index) const {
    int iY = _index / maxx;
    int iX = _index - iY * maxx;
    return Vec(indexXToCoord(iX), indexYToCoord(iY));
  }
  Veci globalIndexToVeci(size_t _index) const {
    int iY = _index / maxx;
    int iX = _index - iY * maxx;
    return Veci(iX, iY);
  }
  size_t VeciToGlobalIndex(Veci _veci) {
    return _veci.y * maxx + _veci.x;
  }
  size_t coordToGlobalIndex(Vec _pos) {
    return VeciToGlobalIndex(vecToVeci(_pos));
  }

  void markVisitedInRadius(const Vec& _pos, float _radius) {
    markGroundTypeRadius(_pos, _radius, gt_visited, {gt_unknown, gt_probably_reachable_seen, gt_probably_reachable_polygon, gt_probably_unreachable});
  }

  void markGroundTypeRadius(const Vec& _pos, float _radius, GroundType _groundType, std::vector<int> _ifInList = std::vector<int>()) {
    unsigned int xOnGround = coordXToIndex(_pos.x);
    unsigned int yOnGround = coordYToIndex(_pos.y);
    bool stop = false;
    int indexradius = 1;
    while (!stop) {
      for (int i = -indexradius; i < indexradius; i++) {
        for (int j = -indexradius; j < indexradius; j++) {
          stop = true; // This seems to be the right place!
          if (validIndexX(xOnGround + i) && validIndexY(yOnGround + j)) {
            if (indicesToCoords(xOnGround + i, yOnGround + j).dist(_pos) <= _radius) {
              stop = false;
              if (!_ifInList.size() || MathUtils::in(groundType(xOnGround + i, yOnGround + j), _ifInList)) {
                groundType(xOnGround + i, yOnGround + j) = _groundType;
              }
            }
          }
        }
      }
      indexradius++;
    }
  }

  void markGroundTypeNearLineRadius(Line _line, float _radius, GroundType _groundType, std::vector<int> _ifInList = std::vector<int>()) {
    unsigned int minX = std::max((int)coordXToIndex(std::min(_line.getStart().x, _line.getEnd().x)) - 1, 0);
    unsigned int maxX = coordXToIndex(std::max(_line.getStart().x, _line.getEnd().x)) + 1;
    unsigned int minY = std::max((int)coordYToIndex(std::min(_line.getStart().y, _line.getEnd().y)) - 1, 0);
    unsigned int maxY = coordYToIndex(std::max(_line.getStart().y, _line.getEnd().y)) + 1;

    // std::cout << "minX: " << minX << ", maxX" << maxX << ", minY" << minY << ", maxY" << maxY << std::endl;

    // markGroundTypeRectangle(minX, minY, maxX, maxY, _groundType);
    // return;

    for (unsigned int x = minX; x <= maxX; x++) {
      for (unsigned int y = minY; y <= maxY; y++) {
        if (validIndexX(x) && validIndexY(y) && _line.distanceToPoint(Vec(indexXToCoord(x), indexYToCoord(y))) <= _radius) {
          if (!_ifInList.size() || MathUtils::in(groundType(x, y), _ifInList))
            groundType(x, y) = _groundType;
        }
      }
    }
  }

  void markGroundTypeRectangle(unsigned int _x, unsigned int _y, unsigned int _width, unsigned int _height, GroundType _groundType, std::vector<int> _ifInList = std::vector<int>()) {
    for (unsigned int i = _x; i < _x + _width; i++) {
      for (unsigned int j = _x; j < _y + _height; j++) {
        if (validIndexX(i) && validIndexY(j)) {
          if (!_ifInList.size() || MathUtils::in(groundType(i, j), _ifInList)) {
            groundType(i, j) = _groundType;
          }
        }
      }
    }
  }

  void markGroundTypeRectangle(float _x, float _y, float _width, float _height, GroundType _groundType, std::vector<int> _ifInList = std::vector<int>()) {
    markGroundTypeRectangle(coordXToIndex(_x), coordYToIndex(_y), coordXToIndex(_x + _width), coordYToIndex(_y + _height), _groundType, _ifInList);
  }

  void markGroundTypePolygons(std::vector<Polygon> _polygons, float _minDistToLine = 0.0f) {
    for (size_t k = 0; k < _polygons.size(); k++) {
      if (_polygons[k].size() < 3)
        continue;
      std::vector<OpenMesh::Vec3f> points;
      for (size_t i = 0; i < _polygons[k].size(); i++) {
        points.push_back(OpenMesh::Vec3f(_polygons[k][i].x, _polygons[k][i].y, 0));
      }
      ACG::Triangulator triangulator(points);
      std::vector<int> indices = triangulator.indices();
      // std::cout << indices.size() << std::endl;
      size_t numTriangles = indices.size() / 3;

      float step = 0.2;

      /*for (float x = -1; x < 7; x += step) {
        for (float y = -1; y < 7; y += step) {
          for (size_t j = 0; j < numTriangles; j++) {
            if (MathUtils::PointInTriangle(Vec(x,y), _polygons[k][indices[j*3]], _polygons[k][indices[j*3+1]], _polygons[k][indices[j*3+2]])) {
              bool found = false;
              for (size_t i = 0; i < pointsInPolygon.size(); i++) {
                if (pointsInPolygon[i].dist(Vec(x,y)) < step/4.0) {
                  pointsInPolygon.erase(pointsInPolygon.begin() + i);
                  found = true;
                  break;
                }
              }
              if (!found)
                pointsInPolygon.push_back(Vec(x,y));
            }
          }
        }
      }*/

      float sum = 0;
      for (size_t j = 0; j < numTriangles; j++)
        sum += MathUtils::triangleArea(_polygons[k][indices[j * 3]], _polygons[k][indices[j * 3 + 1]], _polygons[k][indices[j * 3 + 2]]);

      for (size_t j = 0; j < numTriangles; j++) {
        if (sum < 0.5)
          continue;

        for (unsigned int x = 0; x < maxx; x++) {
          for (unsigned int y = 0; y < maxy; y++) {
            float xx = indexXToCoord(x);
            float yy = indexXToCoord(y);
            if (MathUtils::PointInTriangle(Vec(xx, yy), _polygons[k][indices[j * 3]], _polygons[k][indices[j * 3 + 1]], _polygons[k][indices[j * 3 + 2]])) {
              if (groundType(x, y) == gt_unknown) {
                groundType(x, y) = gt_probably_reachable_polygon;
              }
              else if (groundType(x, y) == gt_probably_reachable_polygon) {
                groundType(x, y) = gt_probably_unreachable;
              }
            }
          }
        }
      }
    }

    /*for (std::vector<Vec>::iterator it = pointsInPolygon.begin(); it != pointsInPolygon.end();) {
      bool found = false;
      for (size_t j = 0; j < knownPoints.size(); j++) {
        if (knownPoints[j].dist(*it) < 0.2) {
          pointsInPolygon.erase(it);
          found = true;
          break;
        }
      }
      if (!found)
        it++;
    }*/

    for (unsigned int x = 0; x < maxx; x++) {
      for (unsigned int y = 0; y < maxy; y++) {
        if (groundType(x, y) == gt_probably_reachable_polygon || groundType(x, y) == gt_probably_unreachable) {
          bool found = false;
          for (size_t j = 0; j < knownPoints.size(); j++) {
            if (knownPoints[j].dist(Vec(indexXToCoord(x), indexYToCoord(y))) < 0.2) {
              if (groundType(x, y) == gt_probably_reachable_polygon)
                groundType(x, y) = gt_probably_unreachable;
              found = true;
              break;
            }
          }
          if (!found) {
            if (groundType(x, y) == gt_probably_unreachable)
              groundType(x, y) = gt_unknown;
          }
        }
      }
    }
  }

  void transferGroundType(GroundType _a, GroundType _b) {
    for (unsigned int x = 0; x < maxx; x++) {
      for (unsigned int y = 0; y < maxy; y++) {
        if (groundType(x, y) == _a)
          groundType(x, y) = _b;
      }
    }
  }

  void resetProbableGroundToUnknown() {
    for (unsigned int x = 0; x < maxx; x++) {
      for (unsigned int y = 0; y < maxy; y++) {
        if (MathUtils::in(groundType(x, y), {gt_probably_reachable_polygon, gt_probably_unreachable}))
          groundType(x, y) = gt_unknown;
      }
    }
  }

  void markGridPointsNearKnownPointsAsWalls(float _radius) {
    for (size_t i = 0; i < knownPoints.size(); i++) {
      markGroundTypeRadius(knownPoints[i], _radius, gt_wall);
    }
  }

  void markGridPointsNearLinesAsWalls(std::vector<Line> _lines, float _radius) {
    for (size_t i = 0; i < _lines.size(); i++) {
      markGroundTypeNearLineRadius(_lines[i], _radius, gt_wall);
    }
  }

  void markGridPointsNearPolygonAsWalls(Polygon _polygon, float _radius) {
    for (size_t i = 0; i < _polygon.size(); i++) {
      markGroundTypeNearLineRadius(Line(_polygon[i], _polygon[(i + 1) % _polygon.size()]), _radius, gt_wall);
    }
  }

  void markGridPointsNearPolygonsAsWalls(std::vector<Polygon> _polygons, float _radius) {
    for (size_t i = 0; i < _polygons.size(); i++) {
      markGridPointsNearPolygonAsWalls(_polygons[i], _radius);
    }
  }

  void localMaximaFilter(float _radius) {
    std::vector<Vec>::iterator it = knownPoints.begin();
    while (it != knownPoints.end()) {
      std::vector<Vec>::iterator it2 = knownPoints.begin();
      while (it2 != knownPoints.end()) {
        if (it != it2 && it->dist(*it2) < _radius) {
          knownPoints.erase(it2);
        }
        else {
          it2++;
        }
      }
      it++;
    }
  }

  // int dijkstra(const vector< vector<edge> >& graph, int source, int target) {
  //   vector<int> min_distance(graph.size(), INT_MAX);
  //   min_distance[ source ] = 0;
  //   set< pair<int, int> > active_vertices;
  //   active_vertices.insert({0, source});

  //   while (!active_vertices.empty()) {
  //     int where = active_vertices.begin()->second;
  //     if (where == target) return min_distance[where];
  //     active_vertices.erase(active_vertices.begin());
  //     for (auto edge : graph[where])
  //       if (min_distance[edge.to] > min_distance[where] + edge.length) {
  //         active_vertices.erase({ min_distance[edge.to], edge.to });
  //         min_distance[edge.to] = min_distance[where] + edge.length;
  //         active_vertices.insert({ min_distance[edge.to], edge.to });
  //       }
  //   }
  //   return INT_MAX;
  // }

  int minDistance(const std::vector<float>& dist, const std::vector<bool>& sptSet) {
    // Initialize min value
    float min = INT_MAX;
    int min_index;

    for (int v = 0; v < dist.size(); v++) {
      if (sptSet[v] == false && dist[v] <= min) {
        min = dist[v];
        min_index = v;
      }
    }

    return min_index;
  }

  // A utility function to print the constructed distance array
  void printSolution(const std::vector<float>& dist, const std::vector<int>& parent) {
    printf("Vertex\t\tDistance from Source\n");
    for (int i = 0; i < dist.size(); i++)
      printf("%d \t\t %.1f \t\t %d\n", i, dist[i] < INT_MAX - 1 ? dist[i] : -1, parent[i]);
  }

  void dijkstra(const Eigen::MatrixXf& _graph, int _from, std::vector<float>& _dist, std::vector<bool>& _sptSet, std::vector<int>& _parent) {
    int maxIndex = _graph.cols();

    // Distance of source vertex from itself is always 0
    _dist[_from] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < maxIndex - 1; count++) {
      // Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src in first iteration.
      int u = minDistance(_dist, _sptSet);

      // Mark the picked vertex as processed
      _sptSet[u] = true;

      // Update dist value of the adjacent vertices of the picked vertex.
      for (int v = 0; v < maxIndex; v++)

        // Update _dist[v] only if is not in sptSet, there is an edge from u to v, and total weight of path from src to  v through u is smaller than current value of _dist[v]
        if (!_sptSet[v] && _graph(u, v) > 0 && _dist[u] < INT_MAX - 1 && _dist[u] + _graph(u, v) < _dist[v]) {
          _dist[v] = _dist[u] + _graph(u, v);
          _parent[v] = u;
        }
    }

    // print the constructed distance array
    // printSolution(_dist, _parent);
  }

  std::vector<size_t> dijkstra_shortestPath(const Eigen::MatrixXf& _graph, int _from, int _to) {
    int maxIndex = _graph.cols();
    std::vector<float> dist(maxIndex, INT_MAX);     // The output array.  dist[i] will hold the shortest distance from src to i
    std::vector<bool> sptSet(maxIndex, false); // sptSet[i] will true if vertex i is included in shortest path tree or shortest distance from src to i is finalized
    std::vector<int> parent(maxIndex, -1);

    dijkstra(_graph, _from, dist, sptSet, parent);

    std::vector<size_t> path;
    if (parent[_to] != -1) {
      int current = _to;
      do {
        path.push_back(current); // globalIndexToVeci(current)
        current = parent[current];
      }
      while (parent[current] != -1);
    }
    // Dont reverse, implemented as stack
    // std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<size_t> dijkstra_shortestPathToReachableUnknown(const Eigen::MatrixXf& _graph, int _from, float _radius) {
    if (!isReachable(globalIndexToVeci(_from))) {
      DEBUG_VAR(_from);
      DEBUG_VAR(globalIndexToVeci(_from));
      std::cout << "!isReachable(globalIndexToVeci(_from))" << std::endl;
      try {
        Veci nearestReachablePoint = findNearestReachablePoint(globalIndexToCoord(_from));
        return { VeciToGlobalIndex(nearestReachablePoint) };
      }
      catch (int e) {
        std::cout << "Exception " << e << " occurred. No reachable point in radius 10 (?)" << std::endl;
        return std::vector<size_t>();
      }
    }

    int maxIndex = _graph.cols();
    std::vector<float> dist(maxIndex, INT_MAX);     // The output array.  dist[i] will hold the shortest distance from src to i
    std::vector<bool> sptSet(maxIndex, false); // sptSet[i] will true if vertex i is included in shortest path tree or shortest distance from src to i is finalized
    std::vector<int> parent(maxIndex, -1);

    dijkstra(_graph, _from, dist, sptSet, parent);

    int indexOfNearestUnknownPoint = -1;

    if (true) {
      float min = INT_MAX;
      for (int v = 0; v < dist.size(); v++) {
        if (dist[v] <= min && dist[v] < INT_MAX - 1) {
          /*int parent_index = parent[v];
          Veci parent_veci = globalIndexToVeci(parent_index);

          GroundType currentGroundType = groundtype(globalIndexToVeci(v));
          GroundType parentGroundType = groundtype(parent_veci);

          if (currentGroundType == gt_unknown && parentGroundType == )*/


          Veci pos = globalIndexToVeci(v);
          bool foundUnknownNeighbor = false;

          // Check for at least one unknown neighbor
          for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
              if ((dx != 0 || dy != 0) && validVeci(Veci(pos.x + dx, pos.y + dy)) && groundType(pos.x + dx, pos.y + dy) == gt_unknown)
                // if (!_fourConnected || dx == 0 || dy == 0)
                foundUnknownNeighbor = true; // Add break;

          if (foundUnknownNeighbor) { // impossible to fullfill:  && isAllReachableInRadius(veciToVec(pos), _radius * 2.0f)
            min = dist[v];
            indexOfNearestUnknownPoint = v;
          }
        }
      }
      // DEBUG_VAR(min);
      // DEBUG_VAR(min_index);
      // DEBUG_VAR(globalIndexToVeci(min_index));
    }
    else { // Find max
      float max = 0;
      for (int v = 0; v < dist.size(); v++) {
        if (dist[v] >= max && dist[v] < INT_MAX - 1) {
          Veci pos = globalIndexToVeci(v);
          bool foundUnknownNeighbor = false;

          // Check for at least one unknown neighbor
          for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
              if ((dx != 0 || dy != 0) && validVeci(Veci(pos.x + dx, pos.y + dy)) && groundType(pos.x + dx, pos.y + dy) == gt_unknown)
                // if (!_fourConnected || dx == 0 || dy == 0)
                foundUnknownNeighbor = true; // Add break;

          if (foundUnknownNeighbor) {
            max = dist[v];
            indexOfNearestUnknownPoint = v;
          }
        }
      }
    }

    if (indexOfNearestUnknownPoint < 0)
      return std::vector<size_t>();

    int _to = indexOfNearestUnknownPoint;

    std::vector<size_t> path;
    if (parent[_to] != -1) {
      int current = _to;
      do {
        path.push_back(current); // globalIndexToVeci(current)
        current = parent[current];
      }
      while (parent[current] != -1);
    }
    // Dont reverse, implemented as stack
    // std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<Vec> VeciPathToVecPath(const std::vector<Veci>& _vecis) {
    std::vector<Vec> vecs;
    for (int i = 0; i < _vecis.size(); i++)
      vecs.push_back(veciToVec(_vecis[i]));
    return vecs;
  }

  std::vector<size_t> VeciPathToGlobalIndexPath(const std::vector<Veci>& _vecis) {
    std::vector<size_t> vecs;
    for (int i = 0; i < _vecis.size(); i++)
      vecs.push_back(VeciToGlobalIndex(_vecis[i]));
    return vecs;
  }

  void prepareConnectivityGraph(Eigen::MatrixXf& _graph, bool _fourConnected) {
    _graph = Eigen::MatrixXf::Zero(maxx * maxy, maxx * maxy);
    for (int x = 0; x < cols(); x++)
      for (int y = 0; y < rows(); y++)
        for (int dx = -1; dx <= 1; dx++)
          for (int dy = -1; dy <= 1; dy++)
            if ((dx != 0 || dy != 0) && validVeci(Veci(x + dx, y + dy)) && isReachable(x, y) && isReachable(x + dx, y + dy))
              if (!_fourConnected || dx == 0 || dy == 0)
                _graph(VeciToGlobalIndex(Veci(x, y)), VeciToGlobalIndex(Veci(x + dx, y + dy))) = std::sqrt(dx * dx + dy * dy);
  }

  std::vector<size_t> shortestPath(Veci _from, Veci _to, bool _fourConnected) {
    std::vector<size_t> path;
    if (isReachable(_from)) {
      Eigen::MatrixXf graph;
      prepareConnectivityGraph(graph, _fourConnected);

      // std::cout << *this << std::endl;
      // std::cout << std::endl;
      // std::cout << graph << std::endl;

      path = dijkstra_shortestPath(graph, VeciToGlobalIndex(_from), VeciToGlobalIndex(_to));
    }
    else {
      try {
        Veci nearestReachablePoint = findNearestReachablePoint(veciToVec(_from));
        path.push_back(VeciToGlobalIndex(nearestReachablePoint));
      }
      catch (int e) {
        std::cout << "Exception " << e << " occurred. No reachable point in radius 10 (?)" << std::endl;
      }
    }
    // DEBUG_VEC(path);
    return path;
  }

  bool unknownNeighbor(Veci _pos) {
    for (int dx = -1; dx <= 1; dx++)
      for (int dy = -1; dy <= 1; dy++)
        if ((dx != 0 || dy != 0) && validVeci(Veci(_pos.x + dx, _pos.y + dy)) && groundType(_pos.x + dx, _pos.y + dy) == gt_unknown)
          // if (!_fourConnected || dx == 0 || dy == 0)
          return true;
    return false;
  }

  bool circumNavigateCornersIfPossible(std::vector<size_t>& _path) {
    // DEBUG_VEC(_path);
    const size_t importantPartSize = 3;
    if (_path.size() < importantPartSize)
      return false;

    for (size_t i = _path.size(); i > importantPartSize; i--) {
      size_t i1 = _path[i - 1];
      size_t i2 = _path[i - 2];
      size_t i3 = _path[i - 3];
      Veci p1 = globalIndexToVeci(i1);
      Veci p2 = globalIndexToVeci(i2);
      Veci p3 = globalIndexToVeci(i3);
      Veci v1 = p2 - p1;
      Veci v2 = p3 - p2;
      if (v1.leftHandPerpendicular() == v2 || v1.rightHandPerpendicular() == v2) {
        bool left = v1.leftHandPerpendicular() == v2;
        // std::cout << "Found turn at " << (i - 1) << " to " << (i - 3) << std::endl;
        if ((left && MathUtils::in(groundType(p1 + v1.leftHandPerpendicular()), {gt_unknown, gt_wall})) || (!left && MathUtils::in(groundType(p1 + v1.rightHandPerpendicular()), {gt_unknown, gt_wall}))) {
          // std::cout << "around unknown" << std::endl;
          if (isReachable(p2 + v1) && isReachable(p2 + v1 + v2)) {
            // Double v1
            _path.erase(_path.begin(), _path.begin() + i - 2); // Delete rest of path and point around corner
            _path.insert(_path.begin(), VeciToGlobalIndex(p2 + v1));
            _path.insert(_path.begin(), VeciToGlobalIndex(p2 + v1 + v2));
            // std::cout << "After: ";
            // DEBUG_VEC(_path);
            return true;
          }
        }
      }
      /*if (v1.rightHandPerpendicular() == v2) {
        std::cout << "Found right turn" << std::endl;
        if (MathUtils::in(groundType(p1 + v1.rightHandPerpendicular()), {gt_unknown, gt_wall}))
          std::cout << "around unknown" << std::endl;
      }*/
    }
    return false;
  }

  std::vector<size_t> getFirstReachableUnknownPoint(Veci _from, float _radius, bool _fourConnected, bool _popLast) {
    Eigen::MatrixXf graph;
    prepareConnectivityGraph(graph, _fourConnected);
    std::vector<size_t> path = dijkstra_shortestPathToReachableUnknown(graph, VeciToGlobalIndex(_from), _radius);
    bool circumNavigatedEdge = circumNavigateCornersIfPossible(path);
    if (!circumNavigatedEdge && _popLast && path.size() > 1)
      path.erase(path.begin());
    // DEBUG_VEC(path);
    return path;
  }

  static void connectToPointsInGraph(Eigen::MatrixXi& _graph, unsigned int _p1, unsigned int _p2, unsigned int _weight = 1) {
    _graph(_p1, _p2) = _weight;
    _graph(_p2, _p1) = _weight;
  }

  std::vector<size_t> computeMSTPathUncleaned(size_t startingPos) {
    Eigen::MatrixXi graphUncleaned  = Eigen::MatrixXi::Zero(maxx * maxy, maxx * maxy);
    for (unsigned int x = 0; x < maxx - 1; x++) {
      for (unsigned int y = 0; y < maxy - 1; y++) {
        if (isReachableButUnvisited(x, y)) {
          if (isReachableButUnvisited(x + 1, y))
            connectToPointsInGraph(graphUncleaned, VeciToGlobalIndex(Veci(x, y)), VeciToGlobalIndex(Veci(x + 1, y)));
          if (isReachableButUnvisited(x, y + 1))
            connectToPointsInGraph(graphUncleaned, VeciToGlobalIndex(Veci(x, y)), VeciToGlobalIndex(Veci(x, y + 1)));
        }
      }
    }
    // for (unsigned int i = 0; i < graphAccessible.cols(); i++) {
    //   for (unsigned int j = 0; j < graphAccessible.cols(); j++) {
    //     if (graphAccessible(i, j) > 0 && graphUncleaned(i, j) == 0) {
    //       // Compute minimal path in graphAccessible and set weight in graphUncleaned to path length
    //     }
    //   }
    // }
    return primMST(graphUncleaned, startingPos);
  }

  std::vector<size_t> computeMSTPathAccessible(size_t startingPos) {
    Eigen::MatrixXi graphAccessible = Eigen::MatrixXi::Zero(maxx * maxy, maxx * maxy);
    for (unsigned int x = 0; x < maxx - 1; x++) {
      for (unsigned int y = 0; y < maxy - 1; y++) {
        if (isReachable(x, y)) {
          if (isReachable(x + 1, y))
            connectToPointsInGraph(graphAccessible, VeciToGlobalIndex(Veci(x, y)), VeciToGlobalIndex(Veci(x + 1, y)));
          if (isReachable(x, y + 1))
            connectToPointsInGraph(graphAccessible, VeciToGlobalIndex(Veci(x, y)), VeciToGlobalIndex(Veci(x, y + 1)));
        }
      }
    }
    return primMST(graphAccessible, startingPos);
  }

  int findSetContaining(std::vector<std::unordered_set<int> >& _sets, int num) {
    for (size_t i = 0; i < _sets.size(); i++) {
      if (_sets[i].find(num) != _sets[i].end())
        return i;
    }
    return -1;
  }

  std::vector<size_t> getRepresentativesOfConnectedRegions() {
    std::vector<std::pair<int, int> > equivalencePairs;
    std::vector<std::unordered_set<int> > equivalenceSets;
    std::vector<int> prelimiaryRepresentatives;
    int regionCounter = 1;
    Eigen::MatrixXi regionLabels = Eigen::MatrixXi::Zero(maxx, maxy);
    for (unsigned int x = 0; x < maxx; x++) {
      for (unsigned int y = 0; y < maxy; y++) {
        if (isReachableButUnvisited(x, y)) {
          bool left = validIndexX(x - 1) && isReachableButUnvisited(x - 1, y);
          bool top = validIndexY(y - 1) && isReachableButUnvisited(x, y - 1);

          if (left && top) {
            regionLabels(x, y) = regionLabels(x - 1, y);
            if (regionLabels(x - 1, y) != regionLabels(x, y - 1)) {
              equivalencePairs.push_back(std::make_pair(regionLabels(x - 1, y), regionLabels(x, y - 1)));
            }
          }
          else if (left) {
            regionLabels(x, y) = regionLabels(x - 1, y);
          }
          else if (top) {
            regionLabels(x, y) = regionLabels(x, y - 1);
          }
          else {
            regionLabels(x, y) = regionCounter;
            int index = VeciToGlobalIndex(Veci(x, y));
            prelimiaryRepresentatives.push_back(index);
            equivalenceSets.push_back({ regionCounter });
            std::cout << "equivalenceSets.push_back({ " << regionCounter << std::endl;
            regionCounter++;
          }
        }
      }
    }

    for (size_t i = 0; i < equivalenceSets.size(); i++) {
      DEBUG_SET(equivalenceSets[i]);
    }

    // DEBUG_MAT(regionLabels);
    // DEBUG_VAR(equivalencePairs.size());
    // DEBUG_VAR(equivalenceSets.size());
    DEBUG_VEC(prelimiaryRepresentatives);

    for (size_t i = 0; i < equivalencePairs.size(); i++) {
      // DEBUG_VAR(equivalencePairs[i].first);
      // DEBUG_VAR(equivalencePairs[i].second);
      int i1 = findSetContaining(equivalenceSets, equivalencePairs[i].first);
      int i2 = findSetContaining(equivalenceSets, equivalencePairs[i].second);
      // DEBUG_VAR(i1);
      // DEBUG_VAR(i2);
      if (i1 != i2) {
        equivalenceSets[i1].insert(equivalenceSets[i2].begin(), equivalenceSets[i2].end());
        equivalenceSets[i2].clear();
      }
    }

    for (size_t i = 0; i < equivalenceSets.size(); i++) {
      DEBUG_SET(equivalenceSets[i]);
    }

    std::vector<size_t> representatives;
    for (size_t i = 0; i < equivalenceSets.size(); i++) {
      if (equivalenceSets[i].size()) {
        representatives.push_back(prelimiaryRepresentatives[*equivalenceSets[i].begin() - 1]); // regionCounter starts with 1!, reduce by one to get index in prelimiaryRepresentatives
      }
    }
    DEBUG_VAR(representatives);
    return representatives;
  }

  std::vector<size_t> computePathOfUncleanedMSTs(size_t startingPos, bool _fourConnected) {
    std::vector<size_t> representatives = getRepresentativesOfConnectedRegions();

    Eigen::MatrixXf graph;
    prepareConnectivityGraph(graph, _fourConnected);

    std::vector<size_t> path;
    size_t currentPos = startingPos;
    for (size_t i = 0; i < representatives.size(); i++) {
      std::vector<size_t> pathToNextRepr = dijkstra_shortestPath(graph, currentPos, representatives[i]);
      std::cout << "Can drive from " << currentPos << " to " << representatives[i] << ": " << pathToNextRepr.size() << std::endl;
      if (representatives[i] > maxx * maxy)
        continue;
      if (pathToNextRepr.size()) {
        path.insert(path.begin(), pathToNextRepr.begin(), pathToNextRepr.end());
        std::vector<size_t> pathi = computeMSTPathUncleaned(representatives[i]);
        path.insert(path.begin(), pathi.begin(), pathi.end());
        currentPos = path.front();
      }
    }
    return path;
  }
};


