#include "curve.h"
#include <math.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>

namespace geometry {

	//---------------------------Macros and global variables---------------------------//
	// These are related to the b-spline
	const float DELTA_T = 0.01f;
	const float DELTA_U = 0.001f; // unit increment of the parameter u
	const int B_SPLINE_ORDER = 3; // order of the b-spline curve
	const int N = 1000;				  // Arc length paramaterization step
	const float DELTA_S = 0.001f;	

	//---------------------------Macros and global variables---------------------------//

	using namespace glm;

	Curve::Curve() {}

	Curve::Curve(Points points) : m_points(points) {
		// concatenate the heading and trailing 3 points
		m_points.insert(m_points.begin(), points.end() - B_SPLINE_ORDER, points.end());
		m_points.insert(m_points.end(), points.begin(), points.begin() + B_SPLINE_ORDER);
		int m = m_points.size() - 1;
		float step = 1.f / (m - B_SPLINE_ORDER + 2);
		for (int i = 0; i < B_SPLINE_ORDER; i++) {
			U.push_back(0);
		}

		for (int i = 1; i <= m - B_SPLINE_ORDER + 1; i++) {
			U.push_back(step*i);
		}

		for (int i = 0; i < B_SPLINE_ORDER; i++) {
			U.push_back(1);
		}
	}

	vec3 Curve::operator[](int idx) const { return m_points[idx]; }

	vec3 &Curve::operator[](int idx) { return m_points[idx]; }

	vec3 Curve::front() const { return m_points.front(); }

	vec3 Curve::back() const { return m_points.back(); }

	size_t Curve::pointCount() const { return m_points.size(); }

	vec3 const *Curve::data() const { return m_points.data(); }

	std::vector<vec3> const &Curve::points() const { return m_points; }

	vec3 Curve::operator()(float u) const {	// C(u)

		int m = m_points.size() - 1;
		// Map the parameter to the standard parameterization 
		float step = 1.f / (m - B_SPLINE_ORDER + 2);
		u = u + (1 - 2 * u) * step * (B_SPLINE_ORDER - 1);
		int d = getDelta(u);
		std::vector<vec3> C;
		for (int i = 0; i < B_SPLINE_ORDER; i++) {
			C.push_back(m_points[d - i]);
		}

		for (int r = B_SPLINE_ORDER; r >= 2; r--) {
			int i = d;
			for (int s = 0; s <= r - 2; s++) {
				float omega = (u - U[i]) / (U[i + r - 1] - U[i]);
				C.at(s) = C.at(s)*omega + (1.f - omega)*C.at(s + 1);
				i--;
			}
		}
		return C.at(0);
	}

	int Curve::getDelta(float u) const {	// get the index of knot
		int m = m_points.size() - 1;
		for (int i = B_SPLINE_ORDER - 1; i < m + B_SPLINE_ORDER - 1; i++) {
			if (u >= U[i] && u < U[i + 1])return i;
		}
		return m + B_SPLINE_ORDER - 2;
	}

	// Compute the 3rd-order b-spline curve from the control points 
	Points Curve::BSplineCurve() {
		Points points;
		// Record the total arc-length
		L = 0;
		vec3 pre = (*this)(0);
		for (float u = 0.f; u <= 1; u += DELTA_U) {
			vec3 E = (*this)(u);
			points.push_back(E);
			L += distance(pre, E);
			pre = E;
		}

		// Generate the lookup table for arc-length parameterization
		arcLengthParameterization();
		return points;
	}

	//---------------------------Arc length---------------------------//
	vec3 Curve::B(float s){
		int ind = floor(s/DELTA_S);
		return (*this)(LUT[ind]);
	}

	void Curve::arcLengthParameterization() {
		// float ds = L / N;
		float ds = DELTA_S;
		float ds_cur = 0;
		float uh = 0;
		while (uh <= 1) {
			vec3 p_cur = (*this)(uh);
			uh += DELTA_U;
			if ((ds_cur + distance((*this)(uh), p_cur)) > ds) {
				float ul = uh - DELTA_U;
				uh = bisectionRefinementLUT(ul, uh, ds, ds_cur, p_cur);
				LUT.push_back(uh);
				ds_cur = 0;
			}
		}
	}

	// refine the u values
	float Curve::bisectionRefinementLUT(float ul, float uh, float ds, float ds_cur, vec3 p_cur) {
		float THRESH = 0.0005f;
		for (int i = 0; i < 20; i++) {	// the number of max iteration can be changed
			float um = (uh + ul) / 2;
			vec3 pm = (*this)(um);
			float dsm = ds_cur + distance(pm, p_cur);
			if (abs(dsm-ds) < THRESH || (uh - ul) / 2 < THRESH) {
				return um;
			}
			if (dsm < ds) {
				ul = um;
			}
			else {
				uh = um;
			}
		}
		return (uh+ul)/2.f;
	}
	//---------------------------Arc length---------------------------//

	//---------------------------Physics---------------------------//
	

	//---------------------------Physics---------------------------//

} // namespace geometry
