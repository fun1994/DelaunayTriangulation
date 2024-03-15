#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>

class Vertex {
public:
	double x;
	double y;
	int incidentEdge;
};

class HalfEdge {
public:
	int origin;
	int twin;
	int prev;
	int next;
	int incidentFace;
};

class Face {
public:
	int outerComponent;
	std::unordered_set<int> innerComponents;
	std::vector<int> containWhichPoints;
};

class DCEL {
public:
	int indexE;
	int indexF;
	std::unordered_map<int, Vertex> V;
	std::unordered_map<int, HalfEdge> E;
	std::unordered_map<int, Face> F;
};
