//------------------------------------------------------------------------------
// A simple example showing how to use the triangle geometry
//------------------------------------------------------------------------------
#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

#include "io.h"
#include "turntable_controls.h"

#include "curve.h"
#include "curvefileio.h"


using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

int main(void) {
	io::GLFWContext windows;
	auto window =
		windows.create(io::Window::dimensions{ 640, 480 }, "Interpolation example");
	window.enableVsync(true);

	auto view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	glClearColor(1.f, 1.f, 1.f, 1.f);

	// CURVE
	// initialize curve
	geometry::Points points{
		givr::vec3f(-1.f, -1.f, 0.f), // a
		givr::vec3f(1.f, -1.f, 0.f),  // b
		givr::vec3f(1.f, 1.f, 0.f),   // c
		givr::vec3f(-1.f, 1.f, 0.f)   // d
	};

	// smoother curve from points
	//geometry::Curve curve = geometry::cubicSubdivideCurve(points, 2);

	// ... or from file
	 //auto curve = geometry::loadCurveFromFile("curve.txt");
	auto curve = geometry::loadCurveFrom_OBJ_File("../curves/myCurve_2.obj");
	//REMOVE
	auto polyline2 = PolyLine<givr::PrimitiveType::LINE_LOOP>();
	for (auto const p : curve.points()) {
		polyline2.push_back(Point(p));
	}
	auto lineStyle2 = GL_Line(Colour(1.0, 0.0, 0.0));
	auto renderableLine2 = givr::createRenderable(polyline2, lineStyle2);

	//curve = geometry::cubicSubdivideCurve(curve.points(), 2);
	auto b_spline_points = curve.BSplineCurve();

	// package geometry
	auto polyline = PolyLine<givr::PrimitiveType::LINES>();
	for (auto const p : b_spline_points) {
		polyline.push_back(Point(p));
	}

	// rendering style
	auto lineStyle = GL_Line(Colour(0.5, 0.0, 1.0));

	// create renderable
	auto renderableLine = givr::createRenderable(polyline, lineStyle);

	// BEAD
	// geometry
	auto sphere = Sphere();
	// style
	auto phong = Phong(Colour(1.f, 1.f, 0.f), LightPosition(10.f, 10.f, 10.f));
	// create renderable
	auto spheres = givr::createInstancedRenderable(sphere, // geometry
		phong); // style

	// Some Constants
	const float g = 9.8f;
	const float DELTA_T = 0.03f;
	const float SCALER = 5;
	const float CONST_V = 2.f;
	const float L = curve.totalLength();
	const float BRAKE_S = L - 3;
	const float BRAKE_V = -0.5f;
	const float H_S = curve.getMAX_H_S();
	const float H = curve.getH();

	float cur_s = 5;	// the current position
	float cur_speed = 0.f;	// the current speed;
	givr::vec3f cur_pos = curve.B(cur_s);
	

	window.run([&](float frameTime) {
		view.projection.updateAspectRatio(window.width(), window.height());
		// default
		draw(renderableLine, view);
		//draw(renderableLine2, view);

		// or apply model matrix
		// givr::mat4f matrix = scale(givr::mat4f{1.f}, givr::vec3f{10.f});
		// draw(renderableLine, view, matrix);

		 //auto matrix_bead = translate(givr::mat4f{ 1.f }, curve(t_bead));
		auto matrix_bead = translate(givr::mat4f{ 1.f }, cur_pos);
		matrix_bead = scale(matrix_bead, givr::vec3f{ 0.1f });

		addInstance(spheres, matrix_bead);

		draw(spheres, view);

		// TODO:
		// Three phases: [0,s_high] (speed = const_v) --> (s_high, s_brake) free fall --> [s_brake, 0] speed x~0 

		// we know the current position cur_pos
		// cur_pos.y --> cur_speed --> cur_accel --> frame
		// cur_speed*DELTA_T --> delta_s --> next_pos
		// cur_pos = next_pos

		
		if (cur_s > H_S&& cur_s < BRAKE_S) {
			// Update the status
			cur_speed = sqrt(2 * g * (H - cur_pos.y));
		}
		else {
			cur_speed = CONST_V;
		}

		cur_s += cur_speed * DELTA_T;
		if (cur_s >= L)
			cur_s = 0.f;

		cur_pos = curve.B(cur_s);
	});

	exit(EXIT_SUCCESS);
}
