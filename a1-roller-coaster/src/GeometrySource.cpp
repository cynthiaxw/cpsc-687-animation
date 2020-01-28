#include "Geometry.h"

#include <cmath>
#define M_PI 3.1416

using namespace glm;
using namespace std;

Geometry::Geometry() {
	drawMode = GL_TRIANGLES;
	vao = 0;
	vertexBuffer = 0;
	colourBuffer = 0;
	modelMatrix = glm::mat4(1.f);
	pointSize = 10.0f;
}

Geometry Geometry::makeCircle(float radius, float uInc) {

	Geometry circle;

	for (double u = 0.0; u <= 2.0 * M_PI; u += uInc) {
		circle.verts.push_back(radius * glm::vec3(cos(u), sin(u), 0));
		circle.colours.push_back(glm::vec3(0.f, 1.f, 0.f));
	}
	circle.drawMode = GL_LINE_STRIP;
	return circle;
}

Geometry Geometry::makePoints(std::vector<glm::vec3> *point, int select_index) {
	Geometry points;
	for (int i = 0; i < point->size(); i++) {
		points.verts.push_back(point->at(i));
		if (i == select_index) {
			points.colours.push_back(glm::vec3(0, 1, 1));
		}
		else points.colours.push_back(glm::vec3(1, 1, 0));
	}
	points.drawMode = GL_POINTS;
	return points;
}

Geometry Geometry::makePolygon(std::vector<glm::vec3> *point) {
	Geometry polygon;
	for (int i = 0; i < point->size(); i++) {
		polygon.verts.push_back(point->at(i));
		polygon.colours.push_back(glm::vec3(0, 0.8, 0.8));
	}
	polygon.drawMode = GL_LINE_STRIP;
	return polygon;
}

int getDelta(vector<float> *U, float u, int k, int m) {	// return the index of knot
	for (int i = k - 1; i < m + k - 1; i++) {
		if (u >= U->at(i) && u < U->at(i + 1))return i;
	}
	return m + k - 2;
}

vec3 E_delta_1_spline(vector<vec3> *E, vector<float> *U, float u, int k, int m, vector<vec3> *geoV, vector<vec3> *geoLine, int flg) {
	int d = getDelta(U, u, k, m);
	vector<vec3> C;
	for (int i = 0; i < k; i++) {
		C.push_back(E->at(d - i));
		if (flg) {
			geoV->push_back(E->at(d - i));
			if (i == 0 || i == k - 1)geoLine->push_back(C.at(i));
			else {
				geoLine->push_back(C.at(i));
				geoLine->push_back(C.at(i));
			}
		}
	}

	for (int r = k; r >= 2; r--) {
		int i = d;
		for (int s = 0; s <= r - 2; s++) {
			float omega = (u - U->at(i)) / (U->at(i + r - 1) - U->at(i));
			C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
			if (flg) {
				geoV->push_back(C.at(s));
				if (s == 0 || s == r - 2)geoLine->push_back(C.at(s));
				else {
					geoLine->push_back(C.at(s));
					geoLine->push_back(C.at(s));
				}
			}
			i--;
		}
	}
	return C.at(0);
}

vec3 E_delta_1(vector<vec3> *E, vector<float> *U, float u, int k, int m) {
	int d = getDelta(U, u, k, m);
	vector<vec3> C;
	for (int i = 0; i < k; i++) {
		C.push_back(E->at(d - i));
	}

	for (int r = k; r >= 2; r--) {
		int i = d;
		for (int s = 0; s <= r - 2; s++) {
			float omega = (u - U->at(i)) / (U->at(i + r - 1) - U->at(i));
			C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
			i--;
		}
	}
	return C.at(0);
}

float W_delta_1(vector<float> *E, vector<float> *U, float u, int k, int m) {
	int d = getDelta(U, u, k, m);
	vector<float> C;
	for (int i = 0; i < k; i++) {
		C.push_back(E->at(d - i));
	}

	for (int r = k; r >= 2; r--) {
		int i = d;
		for (int s = 0; s <= r - 2; s++) {
			float omega = (u - U->at(i)) / (U->at(i + r - 1) - U->at(i));
			C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
			i--;
		}
	}
	return C.at(0);
}

Geometry Geometry::makeNURBS(std::vector<glm::vec3> *P, std::vector<float> *W, float u_inc, int k) {
	Geometry spline;
	int m = P->size() - 1;
	if (m < k - 1)return spline;
	float step = 1.f / (float)(m - k + 2);
	vector<float> U;	//knot sequence
	for (int i = 0; i < k; i++) {
		U.push_back(0);
	}

	for (int i = 1; i <= m - k + 1; i++) {
		U.push_back(step*i);
	}

	for (int i = 0; i < k; i++) {
		U.push_back(1);
	}

	vector<vec3> N;
	for (int i = 0; i < P->size(); i++) {
		N.push_back(P->at(i) * W->at(i));
	}

	for (float u = 0.f; u <= 1; u += u_inc) {
		vec3 E = E_delta_1(&N, &U, u, k, m);
		float w = W_delta_1(W, &U, u, k, m);
		spline.verts.push_back(E / w);
		spline.colours.push_back(vec3(1, 0, 0));
	}

	spline.drawMode = GL_LINE_STRIP;
	return spline;
}

Geometry Geometry::makeSpline(std::vector<glm::vec3> *P, float u_inc, int k, vector<vec3> *geoV, vector<vec3> *geoLine, float u_pos) {
	Geometry spline;
	int m = P->size() - 1;
	if (m < k - 1)return spline;
	float step = 1.f / (float)(m - k + 2);
	vector<float> U;	//knot sequence
	for (int i = 0; i < k; i++) {
		U.push_back(0);
	}

	for (int i = 1; i <= m - k + 1; i++) {
		U.push_back(step*i);
	}

	for (int i = 0; i < k; i++) {
		U.push_back(1);
	}

	for (float u = 0.f; u <= 1; u += u_inc) {
		vec3 E = E_delta_1_spline(P, &U, u, k, m, geoV, geoLine, 0);
		spline.verts.push_back(E);
		spline.colours.push_back(vec3(1, 0, 0));
	}

	E_delta_1_spline(P, &U, u_pos, k, m, geoV, geoLine, 1);

	spline.drawMode = GL_LINE_STRIP;
	return spline;
}

Geometry Geometry::makeSplineGeo(std::vector<glm::vec3> *P, float u, int k) {	// geo points
	Geometry SplineGeo;
	int m = P->size() - 1;
	if (m < k - 1)return SplineGeo;
	float step = 1.f / (float)(m - k + 2);
	vector<float> U;	//knot sequence
	for (int i = 0; i < k; i++) {
		U.push_back(0);
	}

	for (int i = 1; i <= m - k + 1; i++) {
		U.push_back(step*i);
	}

	int d = getDelta(&U, u, k, m);
	cout << "d" << d << endl;
	vector<vec3> C;
	for (int i = 0; i < k; i++) {
		C.push_back(P->at(d - i));
		SplineGeo.verts.push_back(P->at(d - i));
		SplineGeo.colours.push_back(vec3(0, 0, 1));
	}

	for (int r = k; r >= 2; r--) {
		int i = d;
		for (int s = 0; s <= r - 2; s++) {
			float omega = (u - U.at(i)) / (U.at(i + r - 1) - U.at(i));
			C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
			SplineGeo.verts.push_back(C.at(s));
			SplineGeo.colours.push_back(vec3(0, 0, 1));
			i--;
		}
	}

	SplineGeo.drawMode = GL_POINTS;
	SplineGeo.pointSize = 5.f;
	return SplineGeo;
}
