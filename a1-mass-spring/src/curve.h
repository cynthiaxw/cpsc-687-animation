#pragma once

#include <iosfwd>
#include <vector>

#include <glm/glm.hpp>

namespace geometry {

	using Points = std::vector<glm::vec3>;

	class Curve {
	public:
		Curve();
		Curve(Points points);

		glm::vec3 operator[](int idx) const;
		glm::vec3 &operator[](int idx);
		glm::vec3 front() const;
		glm::vec3 back() const;
		glm::vec3 B(float s);	// arc-len parameterization
		int getDelta(float u) const;
		Points BSplineCurve();
		Points setUp();
		void arcLengthParameterization();
		float bisectionRefinementLUT(float ul, float uh, float ds, float ds_cur, glm::vec3 p_cur);
		glm::vec3 bisectionRefinementLUT1(float ul, float uh, float ds, float ds_cur, glm::vec3 p_cur);
		float totalLength();
		float getHighestS();
		float getMAX_H_S();
		float getH();
		
		// C(u)
		glm::vec3 operator()(float t) const;

		size_t pointCount() const;
		glm::vec3 const *data() const;
		Points const &points() const;
		std::vector<float> LUT;
		std::vector<glm::vec3> LUT1;

	private:
		Points m_points;
		std::vector<float> U;
		std::vector<float> UP;
		float L;
		float H;
		float MAX_H_S;
	};

} // namespace geometry
