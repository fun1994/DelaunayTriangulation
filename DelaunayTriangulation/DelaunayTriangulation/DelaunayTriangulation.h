#pragma once
#include <queue>
#include "Point.h"
#include "DCEL.h"

class DelaunayTriangulation {
	void initialize(DCEL& D, std::vector<Point>& P, std::unordered_map<int, int>& inWhichTriangle) {
		double M = abs(P[0].x);
		for (int i = 0; i < P.size(); i++) {
			M = M < abs(P[i].x) ? abs(P[i].x) : M;
			M = M < abs(P[i].y) ? abs(P[i].y) : M;
		}
		Vertex v;
		v.x = 3 * M; v.y = 0; v.incidentEdge = 0; D.V[-1] = v;
		v.x = 0; v.y = 3 * M; v.incidentEdge = 2; D.V[-2] = v;
		v.x = -3 * M; v.y = -3 * M; v.incidentEdge = 4; D.V[-3] = v;
		D.indexE = 0;
		HalfEdge e;
		e.origin = -1; e.twin = 1; e.prev = 4; e.next = 2; e.incidentFace = 0; D.E[D.indexE++] = e;
		e.origin = -2; e.twin = 0; e.prev = 3; e.next = 5; e.incidentFace = -1; D.E[D.indexE++] = e;
		e.origin = -2; e.twin = 3; e.prev = 0; e.next = 4; e.incidentFace = 0; D.E[D.indexE++] = e;
		e.origin = -3; e.twin = 2; e.prev = 5; e.next = 1; e.incidentFace = -1; D.E[D.indexE++] = e;
		e.origin = -3; e.twin = 5; e.prev = 2; e.next = 0; e.incidentFace = 0; D.E[D.indexE++] = e;
		e.origin = -1; e.twin = 4; e.prev = 1; e.next = 3; e.incidentFace = -1; D.E[D.indexE++] = e;
		D.indexF = 0;
		Face f;
		f.outerComponent = -1; f.innerComponents.insert(1); D.F[-1] = f;
		f.outerComponent = 0; f.innerComponents.clear();
		for (int i = 0; i < P.size(); i++) {
			f.containWhichPoints.push_back(i);
		}
		D.F[D.indexF++] = f;
		for (int i = 0; i < P.size(); i++) {
			inWhichTriangle[i] = 0;
		}
	}
	void insert(DCEL& D, std::vector<Point>& P, int index, std::unordered_map<int, int>& inWhichTriangle, std::queue<int>& Q) {
		Vertex v;
		v.x = P[index].x;
		v.y = P[index].y;
		v.incidentEdge = D.indexE;
		D.V[index] = v;
		HalfEdge e;
		e.origin = index;
		e.twin = D.indexE + 1;
		e.prev = D.indexE + 3;
		e.next = D.F[inWhichTriangle[index]].outerComponent;
		e.incidentFace = D.indexF;
		D.E[D.indexE] = e;
		e.origin = D.E[D.F[inWhichTriangle[index]].outerComponent].origin;
		e.twin = D.indexE;
		e.prev = D.E[D.F[inWhichTriangle[index]].outerComponent].prev;
		e.next = D.indexE + 4;
		e.incidentFace = D.indexF + 2;
		D.E[D.indexE + 1] = e;
		e.origin = index;
		e.twin = D.indexE + 3;
		e.prev = D.indexE + 5;
		e.next = D.E[D.F[inWhichTriangle[index]].outerComponent].next;
		e.incidentFace = D.indexF + 1;
		D.E[D.indexE + 2] = e;
		e.origin = D.E[D.E[D.F[inWhichTriangle[index]].outerComponent].next].origin;
		e.twin = D.indexE + 2;
		e.prev = D.F[inWhichTriangle[index]].outerComponent;
		e.next = D.indexE;
		e.incidentFace = D.indexF;
		D.E[D.indexE + 3] = e;
		e.origin = index;
		e.twin = D.indexE + 5;
		e.prev = D.indexE + 1;
		e.next = D.E[D.F[inWhichTriangle[index]].outerComponent].prev;
		e.incidentFace = D.indexF + 2;
		D.E[D.indexE + 4] = e;
		e.origin = D.E[D.E[D.F[inWhichTriangle[index]].outerComponent].prev].origin;
		e.twin = D.indexE + 4;
		e.prev = D.E[D.F[inWhichTriangle[index]].outerComponent].next;
		e.next = D.indexE + 2;
		e.incidentFace = D.indexF + 1;
		D.E[D.indexE + 5] = e;
		D.E[D.E[D.indexE].next].prev = D.indexE;
		D.E[D.E[D.indexE].next].next = D.indexE + 3;
		D.E[D.E[D.indexE].next].incidentFace = D.indexF;
		D.E[D.E[D.indexE + 2].next].prev = D.indexE + 2;
		D.E[D.E[D.indexE + 2].next].next = D.indexE + 5;
		D.E[D.E[D.indexE + 2].next].incidentFace = D.indexF + 1;
		D.E[D.E[D.indexE + 4].next].prev = D.indexE + 4;
		D.E[D.E[D.indexE + 4].next].next = D.indexE + 1;
		D.E[D.E[D.indexE + 4].next].incidentFace = D.indexF + 2;
		Face f;
		f.outerComponent = D.indexE;
		D.F[D.indexF] = f;
		f.outerComponent = D.indexE + 2;
		D.F[D.indexF + 1] = f;
		f.outerComponent = D.indexE + 4;
		D.F[D.indexF + 2] = f;
		for (int i = 0; i < D.F[inWhichTriangle[index]].containWhichPoints.size(); i++) {
			if (D.F[inWhichTriangle[index]].containWhichPoints[i] != index) {
				bool toLeft0 = toLeft(D, D.E[D.indexE], P[D.F[inWhichTriangle[index]].containWhichPoints[i]]);
				bool toLeft1 = toLeft(D, D.E[D.indexE + 2], P[D.F[inWhichTriangle[index]].containWhichPoints[i]]);
				bool toLeft2 = toLeft(D, D.E[D.indexE + 4], P[D.F[inWhichTriangle[index]].containWhichPoints[i]]);
				if (toLeft0 && !toLeft1) {
					D.F[D.indexF].containWhichPoints.push_back(D.F[inWhichTriangle[index]].containWhichPoints[i]);
					inWhichTriangle[D.F[inWhichTriangle[index]].containWhichPoints[i]] = D.indexF;
				}
				else if (toLeft1 && !toLeft2) {
					D.F[D.indexF + 1].containWhichPoints.push_back(D.F[inWhichTriangle[index]].containWhichPoints[i]);
					inWhichTriangle[D.F[inWhichTriangle[index]].containWhichPoints[i]] = D.indexF + 1;
				}
				else {
					D.F[D.indexF + 2].containWhichPoints.push_back(D.F[inWhichTriangle[index]].containWhichPoints[i]);
					inWhichTriangle[D.F[inWhichTriangle[index]].containWhichPoints[i]] = D.indexF + 2;
				}
			}
		}
		D.F.erase(inWhichTriangle[index]);
		inWhichTriangle.erase(index);
		Q.push(D.E[D.indexE].next);
		Q.push(D.E[D.indexE + 2].next);
		Q.push(D.E[D.indexE + 4].next);
		D.indexE += 6;
		D.indexF += 3;
	}
	double area2(Vertex& p, Vertex& q, Point& r) {
		return p.x * q.y - p.y * q.x + q.x * r.y - q.y * r.x + r.x * p.y - r.y * p.x;
	}
	bool toLeft(Vertex& p, Vertex& q, Point& r) {
		return area2(p, q, r) > 0;
	}
	bool toLeft(DCEL& D, HalfEdge& e, Point& v) {
		return toLeft(D.V[e.origin], D.V[D.E[e.twin].origin], v);
	}
	bool inCircle(Vertex& a, Vertex& b, Vertex& c, Vertex& p) {
		return -det(b, c, p) + det(a, c, p) - det(a, b, p) + det(a, b, c) > 0;
	}
	double det(Vertex& a, Vertex& b, Vertex& c) {
		return (pow(a.x, 2) + pow(a.y, 2)) * det(b, c) + (pow(b.x, 2) + pow(b.y, 2)) * det(c, a) + (pow(c.x, 2) + pow(c.y, 2)) * det(a, b);
	}
	double det(Vertex& a, Vertex& b) {
		return a.x * b.y - a.y * b.x;
	}
	void flip(DCEL& D, std::vector<Point>& P, int index, std::unordered_map<int, int>& inWhichTriangle, std::queue<int>& Q) {
		D.V[D.E[index].origin].incidentEdge = D.E[D.E[index].twin].next;
		D.V[D.E[D.E[index].twin].origin].incidentEdge = D.E[index].next;
		HalfEdge e;
		e.origin = D.E[D.E[index].prev].origin;
		e.twin = D.indexE + 1;
		e.prev = D.E[index].next;
		e.next = D.E[D.E[index].twin].prev;
		e.incidentFace = D.indexF;
		D.E[D.indexE] = e;
		e.origin = D.E[D.E[D.E[index].twin].prev].origin;
		e.twin = D.indexE;
		e.prev = D.E[D.E[index].twin].next;
		e.next = D.E[index].prev;
		e.incidentFace = D.indexF + 1;
		D.E[D.indexE + 1] = e;
		D.E[D.E[D.indexE].prev].prev = D.E[D.indexE].next;
		D.E[D.E[D.indexE].prev].next = D.indexE;
		D.E[D.E[D.indexE].prev].incidentFace = D.indexF;
		D.E[D.E[D.indexE].next].prev = D.indexE;
		D.E[D.E[D.indexE].next].next = D.E[D.indexE].prev;
		D.E[D.E[D.indexE].next].incidentFace = D.indexF;
		D.E[D.E[D.indexE + 1].prev].prev = D.E[D.indexE + 1].next;
		D.E[D.E[D.indexE + 1].prev].next = D.indexE + 1;
		D.E[D.E[D.indexE + 1].prev].incidentFace = D.indexF + 1;
		D.E[D.E[D.indexE + 1].next].prev = D.indexE + 1;
		D.E[D.E[D.indexE + 1].next].next = D.E[D.indexE + 1].prev;
		D.E[D.E[D.indexE + 1].next].incidentFace = D.indexF + 1;
		Face f;
		f.outerComponent = D.indexE;
		D.F[D.indexF] = f;
		f.outerComponent = D.indexE + 1;
		D.F[D.indexF + 1] = f;
		for (int i = 0; i < D.F[D.E[index].incidentFace].containWhichPoints.size(); i++) {
			if (toLeft(D, D.E[D.indexE], P[D.F[D.E[index].incidentFace].containWhichPoints[i]])) {
				D.F[D.indexF].containWhichPoints.push_back(D.F[D.E[index].incidentFace].containWhichPoints[i]);
				inWhichTriangle[D.F[D.E[index].incidentFace].containWhichPoints[i]] = D.indexF;
			}
			else {
				D.F[D.indexF + 1].containWhichPoints.push_back(D.F[D.E[index].incidentFace].containWhichPoints[i]);
				inWhichTriangle[D.F[D.E[index].incidentFace].containWhichPoints[i]] = D.indexF + 1;
			}
		}
		for (int i = 0; i < D.F[D.E[D.E[index].twin].incidentFace].containWhichPoints.size(); i++) {
			if (toLeft(D, D.E[D.indexE], P[D.F[D.E[D.E[index].twin].incidentFace].containWhichPoints[i]])) {
				D.F[D.indexF].containWhichPoints.push_back(D.F[D.E[D.E[index].twin].incidentFace].containWhichPoints[i]);
				inWhichTriangle[D.F[D.E[D.E[index].twin].incidentFace].containWhichPoints[i]] = D.indexF;
			}
			else {
				D.F[D.indexF + 1].containWhichPoints.push_back(D.F[D.E[D.E[index].twin].incidentFace].containWhichPoints[i]);
				inWhichTriangle[D.F[D.E[D.E[index].twin].incidentFace].containWhichPoints[i]] = D.indexF + 1;
			}
		}
		D.F.erase(D.E[index].incidentFace);
		D.F.erase(D.E[D.E[index].twin].incidentFace);
		D.E.erase(D.E[index].twin);
		D.E.erase(index);
		Q.push(D.E[D.indexE + 1].prev);
		Q.push(D.E[D.indexE].next);
		D.indexE += 2;
		D.indexF += 2;
	}
	void remove(DCEL& D) {
		std::vector<int> boundary;
		int i = D.E[0].prev;
		while (true) {
			D.F.erase(D.E[i].incidentFace);
			int j;
			if (D.E[i].next < 6) {
				j = D.E[D.E[i].prev].twin;
				D.E.erase(D.E[i].prev);
			}
			else {
				j = D.E[D.E[i].next].twin;
				D.E.erase(D.E[i].next);
				boundary.push_back(D.E[i].prev);
			}
			D.E.erase(i);
			if (D.E.find(j) == D.E.end()) {
				break;
			}
			i = j;
		}
		for (int i = -3; i < 0; i++) {
			D.V.erase(i);
		}
		for (int i = 0; i < 6; i++) {
			D.E.erase(i);
		}
		for (int i = 0; i < boundary.size(); i++) {
			D.V[D.E[boundary[i]].origin].incidentEdge = boundary[i];
			D.E[boundary[i]].prev = boundary[i == boundary.size() - 1 ? 0 : i + 1];
			D.E[boundary[i]].next = boundary[i == 0 ? boundary.size() - 1 : i - 1];
			D.E[boundary[i]].incidentFace = -1;
		}
		D.F[-1].innerComponents.clear();
		D.F[-1].innerComponents.insert(boundary[0]);
	}
public:
	void randomizedIncrementalConstruction(DCEL& D, std::vector<Point>& P) {
		std::unordered_map<int, int> inWhichTriangle;
		initialize(D, P, inWhichTriangle);
		for (int i = 0; i < P.size(); i++) {
			std::queue<int> Q;
			insert(D, P, i, inWhichTriangle, Q);
			while (!Q.empty()) {
				if (0 <= D.E[D.E[Q.front()].twin].incidentFace) {
					if (inCircle(D.V[i], D.V[D.E[Q.front()].origin], D.V[D.E[D.E[Q.front()].twin].origin], D.V[D.E[D.E[D.E[Q.front()].twin].prev].origin])) {
						flip(D, P, Q.front(), inWhichTriangle, Q);
					}
				}
				Q.pop();
			}
		}
		remove(D);
	}
};
