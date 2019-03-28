
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <SDL2_ttf/SDL_ttf.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

// #include "extern/hough/hough.hh"
#include "extern/shortestPaths/shortestPaths.hh"
// #include "extern/dijkstra/dijkstra.hh"

#include <Eigen/Dense>

#include "src/Robot.hh"
#include "src/Strategies/RobotAntelope.hh"
#include "src/Strategies/RobotGoat.hh"
#include "src/Visualization/GL.hh"

//Screen dimension constants
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

const int GROUND_WIDTH = 12;
const int GROUND_HEIGHT = 12;

const float PERCENTAGE_THREASHOLD = 0.99f;

void shortestPathStuff() {
  // Eigen::MatrixXi graph(9, 9);
  // // graph <<  0, 2, 0, 6, 0,
  // //           2, 0, 3, 8, 5,
  // //           0, 3, 0, 0, 7,
  // //           6, 8, 0, 0, 9,
  // //           0, 5, 7, 9, 0;
  // graph <<  0, 1, 0, 0, 1, 0, 0, 0, 0,
  //       1, 0, 1, 0, 0, 1, 0, 0, 0,
  //       0, 1, 0, 1, 0, 0, 0, 0, 0,
  //       0, 0, 1, 0, 0, 0, 0, 0, 0,
  //       1, 0, 0, 0, 0, 1, 1, 0, 0,
  //       0, 1, 0, 0, 1, 0, 0, 1, 0,
  //       0, 0, 0, 0, 1, 0, 0, 1, 0,
  //       0, 0, 0, 0, 0, 1, 1, 0, 1,
  //       0, 0, 0, 0, 0, 0, 0, 1, 0;

  // // Print the solution
  // std::vector<size_t> path = primMST(graph, 0);

  // for (size_t i = 0; i < path.size(); i++) {
  //   std::cout << path[i] << std::endl;
  // }
}

int main(int argc, char* args[]) {

  Ground ground(GROUND_WIDTH, GROUND_HEIGHT, 0.2f);
  Environment environment(Ground(GROUND_WIDTH, GROUND_HEIGHT, 0.1f));
  Robot* robot = new RobotAntelope(&ground, &environment, Vec(3.06, 4.49), 0.1/*Vec(0.5, 0.5), 0.1*//*Vec(0.95, 0.8), 0*/);
  // Robot* robot = new RobotGoat(&environment, Vec(0.95, 0.8), 0);

  if (true) {
    std::vector<Polygon> roomPolygons = { Polygon::generateRoom(), Polygon::generatePillar() };
    robot->environment->initTargetWithPolygons(roomPolygons, robot->getRadius(), "roomAndHall");
  }
  else {
    std::vector<Polygon> roomPolygons = { Polygon::generateRoomExtended(), Polygon::generatePillar() };
    robot->environment->initTargetWithPolygons(roomPolygons, robot->getRadius(), "fullApartement");
  }

  // robot->environment->lines = { Line(Vec(1, 1), Vec(2, 1)), Line(Vec(1, 1), Vec(1, 2)) };

  std::vector<Drawable*> drawables = {robot};
  GL gl(SCREEN_WIDTH, SCREEN_HEIGHT, robot, drawables);

  if (!gl.init()) {
    std::cout << "Failed to initialize!" << std::endl;
    return 1;
  }

  gl.SDLMainLoop();

  gl.close();

  return 0;
}
