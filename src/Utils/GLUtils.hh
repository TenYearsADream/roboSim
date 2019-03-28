#pragma once

#ifdef BUILD_WITH_SDL
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <SDL2_ttf/SDL_ttf.h>

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#endif

#include "../Geometry/Ground.hh"
#include "../Geometry/Line.hh"

namespace GLUtils {

struct Color {
	int r, g, b;
	Color(int _r, int _g, int _b) : r(_r), g(_g), b(_b) {}
	void activate() const {
#ifdef BUILD_WITH_SDL
		glColor3f((float)r / 255.0f, (float)g / 255.0f, (float)b / 255.0f);
#endif
	}
};

static Color White = Color(255, 255, 255);
static Color Black = Color(0, 0, 0);
static Color Red = Color(255, 0, 0);
static Color Green = Color(0, 255, 0);
static Color Blue = Color(0, 0, 255);
static Color Turquoise = Color(0, 255, 255);
static Color Yellow = Color(255, 255, 0);

static void setColor(const Color& _color) {
#ifdef BUILD_WITH_SDL
	_color.activate();
#endif
}

static void drawCircle(const Color& _color, const Vec& _center, float _radius, int _linesegments = 36) {
#ifdef BUILD_WITH_SDL
	_color.activate();

	glBegin(GL_LINE_STRIP);
	for (int i = 0; i <= _linesegments; i++) {
		float angle = 2.0f * M_PI / (float)_linesegments * (float)i;
		glVertex2f(_center.x + _radius * sin(angle), _center.y + _radius * cos(angle));
	}
	glEnd();
#endif
}

static void drawLine(const Color& _color, const Vec& _p1, const Vec& _p2) {
#ifdef BUILD_WITH_SDL
	_color.activate();

	glBegin(GL_LINES);
	glVertex2f(_p1.x, _p1.y);
	glVertex2f(_p2.x, _p2.y);
	glEnd();
#endif
}

static void drawLine(const Color& _color, const Line& _line) {
#ifdef BUILD_WITH_SDL
	drawLine(_color, _line.getStart(), _line.getEnd());
#endif
}

static void drawLines(const Color& _color, const std::vector<Line>& _lines) {
#ifdef BUILD_WITH_SDL
	for (size_t i = 0; i < _lines.size(); i++) {
		drawLine(_color, _lines[i]);
	}
#endif
}

static void drawLines(const Color& _color, const std::vector<Vec>& _points, bool _closed = false) {
#ifdef BUILD_WITH_SDL
	_color.activate();

	glBegin(GL_LINE_STRIP);
	for (size_t i = 0; i <= _points.size(); i++) {
		glVertex2f(_points[i % _points.size()].x, _points[i % _points.size()].y);
	}
	glEnd();
#endif
}

static void drawCross(const Color& _color, const Vec& _center, float _radius) {
#ifdef BUILD_WITH_SDL
	_color.activate();

	glBegin(GL_LINES);
	glVertex2f(_center.x - _radius, _center.y - _radius);
	glVertex2f(_center.x + _radius, _center.y + _radius);
	glVertex2f(_center.x + _radius, _center.y - _radius);
	glVertex2f(_center.x - _radius, _center.y + _radius);
	glEnd();
#endif
}

static void drawCrosses(const Color& _color, const std::vector<Vec>& _points, float _radius) {
#ifdef BUILD_WITH_SDL
	for (size_t i = 0; i < _points.size(); i++) {
		drawCross(_color, _points[i], _radius);
	}
#endif
}

static void drawPolygon(const Color& _color, const std::vector<Vec>& _points) {
#ifdef BUILD_WITH_SDL
	drawLines(_color, _points, true);
#endif
}

static void drawPath(const Color& _color, const std::vector<size_t>& _pathPointIndices, const Ground* ground) {
#ifdef BUILD_WITH_SDL
	if (!_pathPointIndices.size() || !ground)
		return;

	for (size_t i = 0; i < _pathPointIndices.size() - 1; i++) {
		Vec pos = ground->globalIndexToCoord(_pathPointIndices[i]);
		Vec pos2 = ground->globalIndexToCoord(_pathPointIndices[i + 1]);
		drawLine(_color, pos, pos2);
	}
#endif
}

static Vec getSelectedPos(int x, int y) {
#ifdef BUILD_WITH_SDL
	GLint vp_f[4];
	glGetIntegerv(GL_VIEWPORT, vp_f);
	Eigen::Vector4i vp(vp_f);
	// matrix_t mv, p;
	float mv_f[16], p_f[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, mv_f);
	glGetFloatv(GL_PROJECTION_MATRIX, p_f);
	Eigen::Matrix4f mv(mv_f);
	Eigen::Matrix4f p(p_f);

	Eigen::Matrix4f inv = (p * mv).inverse();
	float unit_x = (2.0f * ((float)(x - vp[0]) / (vp[2] - vp[0]))) - 1.0f;
	float unit_y = (2.0f * ((float)(y - vp[1]) / (vp[3] - vp[1]))) - 1.0f;
	unit_y *= -1.0f;

	Eigen::Vector4f near = inv * Eigen::Vector4f(unit_x, unit_y, -1, 1);
	Eigen::Vector4f far = inv * Eigen::Vector4f(unit_x, unit_y, 1, 1);
	near /= near.w();
	far /= far.w();

	MathUtils::Vec3f intersection = MathUtils::linePlaneIntersection(MathUtils::Vec3f(0, 0, 0), MathUtils::Vec3f(1, 0, 0), MathUtils::Vec3f(0, 1, 0), near.head(3), far.head(3));

	Vec selectedPos = Vec(intersection.x(), intersection.y());
	return selectedPos;
#endif
}

}



