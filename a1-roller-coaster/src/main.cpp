//------------------------------------------------------------------------------
// A simple example showing how to use the triangle geometry
//------------------------------------------------------------------------------
#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

#include "io.h"
#include "turntable_controls.h"

#include "curve.h"
#include "curvefileio.h"
#include <iostream>


using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using namespace std;

// Global Constants
const float g = 9.81f;
const float DELTA_T = 0.02f;
const float CONST_V = 2.5f;
const float BRAKE_LEN = 20.f;
const float DELTA_S = 0.1f;
const float BAR_LEN = 0.5f;
const float CART_DIST = 1.7f;


mat4 transformMatrix(float cur_speed, float DELTA_S, geometry::Curve curve, float cur_s, vec3 cur_pos){
	vec3 N = cur_speed * cur_speed / (DELTA_S * DELTA_S) * (curve.B(cur_s + DELTA_S) - 2.f * curve.B(cur_s) + curve.B(cur_s - DELTA_S));
	N = N - g * vec3(0, -1, 0);
	N = normalize(N);
	vec3 T = 1.f / DELTA_S * (curve.B(cur_s + DELTA_S) - curve.B(cur_s));
	T = normalize(T);
	vec3 B = normalize(cross(T, N));
	N = normalize(cross(B, T));

	auto m = translate(givr::mat4f{1.f}, cur_pos);
	m = mat4(T[0], T[1], T[2], 0.f,
					   N[0], N[1], N[2], 0.f,
					   B[0], B[1], B[2], 0.f,
					   cur_pos[0], cur_pos[1], cur_pos[2], 1.f);

	m = scale(m, givr::vec3f{1.f});

	return m;
}

int main(void) {
	io::GLFWContext windows;
	auto window =
		windows.create(io::Window::dimensions{ 640, 480 }, "Interpolation example");
	window.enableVsync(true);

	auto view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	glClearColor(1.f, 1.f, 1.f, 1.f);

	// Load curve from file
	auto curve = geometry::loadCurveFrom_OBJ_File("../curves/myCurve.obj");
	auto b_spline_points = curve.BSplineCurve();	// this should be called 

	// BEAD
	// geometry
	auto train_head = givr::geometry::Mesh(Filename("../trains/train_head.obj"));
	auto cart = givr::geometry::Mesh(Filename("../trains/train_tail.obj"));
	// auto sphere = Sphere();
	// style
	auto phong = Phong(Colour(1.f, 1.f, 0.f), LightPosition(10.f, 10.f, 10.f));
	// create renderable
	auto renderable_train_head = givr::createInstancedRenderable(train_head, // geometry
		phong); // style

	auto renderable_cart1 = givr::createInstancedRenderable(cart,
		phong);



	// -------------- Constants and variables --------------//
	const float H_S = curve.getMAX_H_S();
	const float H = curve.getH();
	const float L = curve.totalLength();
	const float BRAKE_S = L - BRAKE_LEN;
	const float BAR_INTERVAL = L / 400;
	
	// Reference Frame
	vec3 N = vec3(0,1,0);	//y
	vec3 T = vec3(1,0,0);	//x
	vec3 B = vec3(0,0,1);	//TxN

	// Reference Frame for cart1
	vec3 N1 = vec3(0,1,0);	//y
	vec3 T1 = vec3(1,0,0);	//x
	vec3 B1 = vec3(0,0,1);	//TxN

	float cur_s = 0;	// the current position
	float cur_speed = 0.f;	// the current speed;
	givr::vec3f cur_pos = curve.B(cur_s);

	// -------------- Constants and variables --------------//

	// calculate the deceleration
	vec3 brake_pos = curve.B(BRAKE_S);
	float brake_speed = sqrt(2 * g * (H - brake_pos.y));
	float deceleration = (brake_speed - CONST_V) / BRAKE_LEN;

	geometry::Points bars;
	geometry::Points track1;
	geometry::Points track2;
	float accum_s = 0.f;
	while (cur_s <= L) {
		N = cur_speed * cur_speed / (DELTA_S*DELTA_S) * (curve.B(cur_s + DELTA_S) - 2.f*curve.B(cur_s) + curve.B(cur_s - DELTA_S));
		N = N - g * vec3(0, -1, 0);
		N = normalize(N);
		T = 1.f / DELTA_S * (curve.B(cur_s + DELTA_S) - curve.B(cur_s));
		T = normalize(T);
		B = normalize(cross(T, N));

		// Update speed and pos
		if (cur_s > H_S + 0.01&& cur_s < BRAKE_S) {
			// Update the status
			cur_speed = sqrt(2 * g * (H - cur_pos.y));
		}
		else if (cur_s >= BRAKE_S && cur_s < L) {
			if (cur_speed < CONST_V){
				cur_speed = CONST_V;
			}else{
				cur_speed -= cur_speed * DELTA_T * deceleration;
			}
		}
		else {
			cur_speed = CONST_V;
		}

		cur_s += cur_speed * DELTA_T;

		accum_s += cur_speed * DELTA_T;//distance(cur_pos, curve.B(cur_s));
		if (accum_s > BAR_INTERVAL) {
			accum_s = 0.f;
			bars.push_back(cur_pos + BAR_LEN * B);
			bars.push_back(cur_pos - BAR_LEN * B);
			track1.push_back(cur_pos + BAR_LEN * B);
			track2.push_back(cur_pos - BAR_LEN * B);
		}

		cur_pos = curve.B(cur_s);
	}

	// package geometry
	auto polyline = PolyLine<givr::PrimitiveType::LINES>();
	for (auto const p : bars) {
		polyline.push_back(Point(p));
	}
	// rendering style
	auto lineStyle = GL_Line(Colour(0.5, 0.0, 1.0));
	// create renderable
	auto renderableBars = givr::createRenderable(polyline, lineStyle);

	// package geometry
	auto polyline1 = PolyLine<givr::PrimitiveType::LINE_LOOP>();
	for (auto const p : track1) {
		polyline1.push_back(Point(p));
	}
	// create renderable
	auto renderableTrack1 = givr::createRenderable(polyline1, lineStyle);

	// package geometry
	auto polyline2 = PolyLine<givr::PrimitiveType::LINE_LOOP>();
	for (auto const p : track2) {
		polyline2.push_back(Point(p));
	}
	// create renderable
	auto renderableTrack2 = givr::createRenderable(polyline2, lineStyle);

	// Re-initialize variables
	cur_s = 0;	// the current position
	cur_speed = 0.f;	// the current speed;
	cur_pos = curve.B(cur_s);

	// Reference Frame
	N = vec3(0, 1, 0);	//y
	T = vec3(1, 0, 0);	//x
	B = vec3(0, 0, 1);	//TxN

	window.run([&](float frameTime) {
		view.projection.updateAspectRatio(window.width(), window.height());
		// Draw the track
		draw(renderableBars, view);
		draw(renderableTrack1, view);
		draw(renderableTrack2, view);

		// or apply model matrix
		// givr::mat4f matrix = scale(givr::mat4f{1.f}, givr::vec3f{10.f});
		// draw(renderableLine, view, matrix);

		// Update T, N, B
		auto matrix_head = transformMatrix(cur_speed, DELTA_S, curve, cur_s, cur_pos);
		addInstance(renderable_train_head, matrix_head);
		draw(renderable_train_head, view);

		auto matrix_cart1 = transformMatrix(cur_speed, DELTA_S, curve, cur_s - CART_DIST, curve.B(cur_s - CART_DIST));
		addInstance(renderable_cart1, matrix_cart1);
		draw(renderable_cart1, view);

		matrix_cart1 = transformMatrix(cur_speed, DELTA_S, curve, cur_s - CART_DIST, curve.B(cur_s - 2*CART_DIST));
		addInstance(renderable_cart1, matrix_cart1);
		draw(renderable_cart1, view);

		matrix_cart1 = transformMatrix(cur_speed, DELTA_S, curve, cur_s - CART_DIST, curve.B(cur_s - 3*CART_DIST));
		addInstance(renderable_cart1, matrix_cart1);
		draw(renderable_cart1, view);

		// Update speed and pos
		if (cur_s > H_S + 0.01&& cur_s < BRAKE_S) {
			// Update the status
			cur_speed = sqrt(2 * g * (H - cur_pos.y));
		}
		else if (cur_s >= BRAKE_S && cur_s < L) {
			if (cur_speed < CONST_V){
				cur_speed = CONST_V;
			}else{
				cur_speed -= cur_speed * DELTA_T * deceleration;
			}
		}
		else {
			cur_speed = CONST_V;
		}

		cur_s += cur_speed * DELTA_T;

		if (cur_s > L)
			cur_s = 0.f;

		cur_pos = curve.B(cur_s);
	});

	exit(EXIT_SUCCESS);
}
