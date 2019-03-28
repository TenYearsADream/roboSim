#include "common.h"

#pragma once

class Point{ 
public:
	float** adjacentMatrix;
	int pointNum; //the number of generated pointset
	set< pair<int,int> > pointset;

	Point();
	~Point();

	void generatePoint(unsigned int H, unsigned int W, unsigned int N);
	set< pair<int,int> > getPointset();
	void setPointset();
	float ** getAdjacentMatrix();
	int getPointNum();
	void printPointset();

private:
	float getEuclideanDistance(int x1, int y1, int x2, int y2);
};