#include "point.h"

Point::Point() {
	pointNum = 0;
}

Point::~Point() {

}

void Point::generatePoint(unsigned int H, unsigned int W, unsigned int N) {
	int row=0, col=0;
	
	if(N > MAX_N || W > MAX_N || H > MAX_N) {
		cout<<"W,H,N should be less than "<<MAX_N<<endl;

		return;
	} else if(N > W * H) {
		cout<<"N should be less than H * W"<<endl;

		return;
	}

	pointNum = N;

	//uniform distrubution generation
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

	std::default_random_engine generator(seed);
	std::uniform_int_distribution<int> x_distribution(0,H);
	std::uniform_int_distribution<int> y_distribution(0,W);

	while(pointset.size() <= N-1) {
		pair<int,int> point(x_distribution(generator), y_distribution(generator));
		pointset.insert(point);
	}
	
	adjacentMatrix = (float**)calloc(N, sizeof(float *));

	for(int i=0; i<N ; ++i) {
		adjacentMatrix[i] = (float*)calloc(N, sizeof(float));
	}

	for(set< pair<int, int> >::iterator it = pointset.begin() ; it != pointset.end() ; ++it, ++row) {
		int x1 = (*it).first;
		int y1 = (*it).second;

		col = 0;
		for(set<pair<int, int>>::iterator that = pointset.begin() ; that != pointset.end() ; ++that, ++col) {
			int x2 = (*that).first;
			int y2 = (*that).second;

			adjacentMatrix[row][col] = getEuclideanDistance(x1, y1, x2, y2);
		}

		if(row % 100 == 0) { //show progress
			cout<<".";
		}
	}

	cout<<endl;
}

void Point::printPointset() {
	 cout<<"Generated pointset list: "<<endl;
	 for(set< pair<int,int> >::iterator it = pointset.begin() ; it != pointset.end() ; ++it) {
		 cout<<(*it).first<<" , "<<(*it).second<<endl;
	 }
	 cout<<endl;

	 cout<<"adjacency matrix: "<<endl;
	 for(int i=0; i<pointNum ; ++i) {
		 for(int j=0; j<pointNum ; ++j) {
			 cout<<adjacentMatrix[i][j]<<"  ";
		 }
		 cout<<endl;
	 }
}

float Point::getEuclideanDistance(int x1, int y1, int x2, int y2) {
	
	return sqrt(pow((double)(x1-x2),2) + pow((double)(y1-y2),2));
}

set< pair<int,int> > Point::getPointset() {

	return pointset;
}

float** Point::getAdjacentMatrix() {

	return adjacentMatrix;
}

int Point::getPointNum() {

	return pointNum;
}
