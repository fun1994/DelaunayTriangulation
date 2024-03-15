#include "DelaunayTriangulation.h"

void generatePoints(std::vector<Point>& P) {
	P.resize(11);
	P[0].x = 172; P[0].y = -129;
	P[1].x = 63; P[1].y = -13;
	P[2].x = 103; P[2].y = -194;
	P[3].x = 76; P[3].y = -281;
	P[4].x = 11; P[4].y = -102;
	P[5].x = 282; P[5].y = -125;
	P[6].x = 211; P[6].y = -80;
	P[7].x = 66; P[7].y = -166;
	P[8].x = 224; P[8].y = -195;
	P[9].x = 228; P[9].y = -27;
	P[10].x = 85; P[10].y = -65;
}

int main() {
	DelaunayTriangulation DT;
	DCEL D;
	std::vector<Point> P;
	generatePoints(P);
	DT.randomizedIncrementalConstruction(D, P);
	return 0;
}
