#include "curve.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>

namespace geometry {

	//---------------------------Macros and global variables---------------------------//
// These are related to the b-spline
	const float DELTA_T = 0.01f;
	const float DELTA_S = 0.005f;
	const float DELTA_U = 0.001f;
	const int B_SPLINE_ORDER = 3;

	//---------------------------Macros and global variables---------------------------//

	using namespace glm;

	// REMOVE
	vec3 midpoint(vec3 const &a, vec3 const &b) { return lerp(a, b, 0.5f); }

	Curve::Curve() {}

	Curve::Curve(Points points) : m_points(points) {}

	vec3 Curve::operator[](int idx) const { return m_points[idx]; }

	vec3 &Curve::operator[](int idx) { return m_points[idx]; }

	vec3 Curve::front() const { return m_points.front(); }

	vec3 Curve::back() const { return m_points.back(); }

	vec3 Curve::operator()(float t) const {	// C(u)
	  // TODO fill will better function
		if (pointCount() == 0)
			return vec3(0.f);

		if (pointCount() == 1)
			return front();

		if (t >= 1.f)
			return front();

		// For now just find closest point
		auto index = t * pointCount();
		return m_points[index];
	}

	vec3 Curve::arcLengthParameterization(float s) const {	// B(s)

	}

	size_t Curve::pointCount() const { return m_points.size(); }

	vec3 const *Curve::data() const { return m_points.data(); }

	std::vector<vec3> const &Curve::points() const { return m_points; }

	// Free functions
	float length(Curve const &curve) {
		float accum_length = 0.f;
		int numLineSegments = curve.pointCount() - 1;

		for (int i = 0; i < numLineSegments; ++i) {
			accum_length += distance(curve[i], curve[i + 1]);
		}

		// the wrap around
		accum_length += distance(curve.front(), curve.back());

		return accum_length;
	}

	int getDelta(std::vector<float> *U, float u, int k, int m) {	// get the index of knot
		for (int i = k - 1; i < m + k - 1; i++) {
			if (u >= U->at(i) && u < U->at(i + 1))return i;
		}
		return m + k - 2;
	}

	vec3 BSplinePosition(Points *control_points, float u) {	// C(u)

	}

	Curve BSplineCurve(Curve const &curve, int order = 3) {
		Points points = curve.points();
		const float u_inc = 0.001f;		// unit increment of the parameter u
		const int order = 3;			// order of the b-spline curve


		return { points };
	}


	// REMOVE
	Curve cubicSubdivideCurve(Curve const &curve, int numberOfSubdivisionSteps) {
		Points points = curve.points();

		for (int iter = 0; iter < numberOfSubdivisionSteps; ++iter) {
			// step 1: subdivide
			points = midpointSubdivide(points);
			// step 2: repeated averageing (2x)
			points = repeatedAveraging(points, 2);
		}
		return { points };
	}

	// REMOVE
	Points midpointSubdivide(Points const &points) {
		Points tmp;
		tmp.reserve(points.size() * 2);
		int numLineSegments = points.size() - 1;

		for (int i = 0; i < numLineSegments; ++i) {
			vec3 mid = lerp(points[i], points[i + 1], 0.5f);
			tmp.push_back(points[i]);
			tmp.push_back(mid);
		}

		vec3 mid = midpoint(points.back(), points.front());
		tmp.push_back(points.back());
		tmp.push_back(mid);

		return tmp;
	}

	// REMOVE
	Points repeatedAveragingStep(Points points) {
		int numLineSegments = points.size() - 1;
		auto front = points.front(); // saved for wrap around calculation

		for (int i = 0; i < numLineSegments; ++i) {
			points[i] = midpoint(points[i], points[i + 1]);
		}
		points.back() = midpoint(points.back(), front);

		return points;
	}

	// REMOVE
	Points repeatedAveraging(Points points, int numberOfAveragingSteps) {
		for (int avgItr = 0; avgItr < numberOfAveragingSteps; ++avgItr) {
			points = repeatedAveragingStep(points);
		}

		return points;
	}

} // namespace geometry
