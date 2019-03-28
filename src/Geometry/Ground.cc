
#include "Ground.hh"
#include "../Utils/GLUtils.hh"
#include "../Config.hh"

void Ground::draw() {
  for (unsigned int x = 0; x < maxx; x++) {
    for (unsigned int y = 0; y < maxy; y++) {
#ifdef BUILD_WITH_SDL
      if (groundType(x, y) == gt_unknown)
        glColor3f(0.5, 0.5, 0.5);
      else if (groundType(x, y) == gt_visited)
        glColor3f(0.0, 1.0, 0.0);
      else if (groundType(x, y) == gt_probably_reachable_seen)
        glColor3f(0.0, 0.0, 1.0);
      else if (groundType(x, y) == gt_probably_reachable_polygon)
        glColor3f(0.0, 0.0, 0.5);
      else if (groundType(x, y) == gt_probably_unreachable)
        glColor3f(1.0, 0.0, 0.0);
      else if (groundType(x, y) == gt_wall)
        glColor3f(1.0, 0.0, 0.0);

      float rad = 0.06;
      glBegin(GL_LINES);
      glVertex2f(indexXToCoord(x) - rad, indexYToCoord(y) - rad);
      glVertex2f(indexXToCoord(x) + rad, indexYToCoord(y) + rad);
      glVertex2f(indexXToCoord(x) + rad, indexYToCoord(y) - rad);
      glVertex2f(indexXToCoord(x) - rad, indexYToCoord(y) + rad);
      glEnd();
#endif
    }
  }
  GLUtils::drawCrosses(Config::color_knownPoints, knownPoints, 0.02);
}