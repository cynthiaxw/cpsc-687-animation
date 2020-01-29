#include "curve.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>

namespace geometry {

	//---------------------------Macros and global variables---------------------------//
	// These are related to the b-spline
	const float DELTA_T = 0.01f;
	const float DELTA_S = 0.005f;
	const float DELTA_U = 0.001f; // unit increment of the parameter u
	const int B_SPLINE_ORDER = 3; // order of the b-spline curve

	//---------------------------Macros and global variables---------------------------//

	using namespace glm;

	// REMOVE
	vec3 midpoint(vec3 const &a, vec3 const &b) { return lerp(a, b, 0.5f); }

	Curve::Curve() {}

	Curve::Curve(Points points) : m_points(points) {
		
		// concatenate the heading and trailing 3 points
		m_points.insert(m_points.begin(), points.end() - B_SPLINE_ORDER+1, points.end());
		m_points.insert(m_points.end(), points.begin(), points.begin() + B_SPLINE_ORDER-1);
		int m = m_points.size() - 1;
		// The b-spline curve should be interpolate the first and the last points
		float step = 1.f / (m+B_SPLINE_ORDER);
		for (int i = 0; i < m + B_SPLINE_ORDER + 1; i++) {
			U.push_back(step*i);
		}

		//float step = 1.f / (float)(m - 3 * B_SPLINE_ORDER + 3);
		//for (int i = 0; i < 2*(B_SPLINE_ORDER-1)+1; i++) {
		//	U.push_back(0);
		//}

		//for (int i = 1; i <= m-3*B_SPLINE_ORDER+3; i++) {
		//	U.push_back(step*i);
		//}

		//for (int i = 0; i < 2*(B_SPLINE_ORDER-1)+1; i++) {
		//	U.push_back(1);
		//}
	}

	vec3 Curve::operator[](int idx) const { return m_points[idx]; }

	vec3 &Curve::operator[](int idx) { return m_points[idx]; }

	vec3 Curve::front() const { return m_points.front(); }

	vec3 Curve::back() const { return m_points.back(); }

	size_t Curve::pointCount() const { return m_points.size(); }

	vec3 const *Curve::data() const { return m_points.data(); }

	std::vector<vec3> const &Curve::points() const { return m_points; }

	vec3 Curve::operator()(float u) const {	// C(u)

		// TODO: if u exceeds [0,1]

		int d = getDelta(u);
		std::vector<vec3> C;
		for (int i = B_SPLINE_ORDER-1; i >= 0; i--) {
			C.push_back(m_points[d + i]);
		}

		//for (int r = B_SPLINE_ORDER; r >= 2; r--) {
		//	int i = d+ B_SPLINE_ORDER - 1;
		//	for (int s = 0; s <= r - 2; s++) {
		//		float omega = (u - U[i]) / (U[i + r - 1] - U[i]);
		//		C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
		//		i--;
		//	}
		//}

		//for (int i = 0; i <B_SPLINE_ORDER; i++) {
		//	C.push_back(m_points[d - i]);
		//}

		for (int r = B_SPLINE_ORDER; r >= 2; r--) {
			int i = d + B_SPLINE_ORDER - 1;
			for (int s = 0; s <= r - 2; s++) {
				float omega = (u - U[i]) / (U[i + r - 1] - U[i]);
				C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
				i--;
			}
		}
		return C.at(0);
	}

	vec3 Curve::arcLengthParameterization(float s) const {	// B(s)
		return vec3(0, 0, 0);
	}

	int Curve::getDelta(float u) const {	// get the index of knot
		//int m = m_points.size() - 1;
		/*for (int i = B_SPLINE_ORDER - 1; i < m + B_SPLINE_ORDER - 1; i++) {
			if (u >= U[i] && u < U[i + 1])return i;
		}
		return m + B_SPLINE_ORDER - 2;*/
		for (int i = 0; i < U.size()-1; i++) {
			if (u >= U[i] && u < U[i + 1])return i;
		}
		return U.size() - 1;
	}

	// Compute the 3rd-order b-spline curve from the control points 
	Points Curve::BSplineCurve() const {
		Points points;
		for (float u = 0.f; u <= 1; u += DELTA_U) {
			vec3 E = (*this)(u);
			points.push_back(E);
		}
		return points;
	}


	// REMOVE
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
