//------------------------------------------------------------------------------
// A simple example showing how to use the triangle geometry
//------------------------------------------------------------------------------
#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>

#include "io.h"
#include "turntable_controls.h"

#include <iostream>


using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using namespace std;

// Global Constants
const float g = 9.81f;
const float Dt = 0.005f;
const int N_BOIDS = 50;
const float BOUNDARY = 10.f;

// tunable parameters, sum to 1
float k_av = 0.005f;
float k_al = 0.4f;
float k_co = 1.f - k_av - k_al;

float k_bd = 0.8f;

typedef struct {
	vec3 pos;	// position
	vec3 f;		// net force
	vec3 v;		// velocity
} Boid;

vec3 force_avoidance(float d, vec3 x) {
	d = std::min(d, 0.01f);
	return k_av/d * normalize(x);
}

vec3 force_alignment(float d, vec3 vi, vec3 vj) {
	return k_al * (vj - vi);
}

vec3 force_cohesion(float d, vec3 xi, vec3 xj) {
	return k_co * (xj-xi);
}

vec3 force_boundary(Boid b) {
	//vec3 f = vec3(0.f);
	//if (b.pos.x < -BOUNDARY) {
	//	f.x += (-BOUNDARY - b.pos.x);
	//}
	//else if (b.pos.x > BOUNDARY) {
	//	f.x += (b.pos.x - BOUNDARY);
	//}

	//if (b.pos.y < -BOUNDARY) {
	//	f.y += (-BOUNDARY - b.pos.y);
	//}
	//else if (b.pos.y > BOUNDARY) {
	//	f.y += (b.pos.y - BOUNDARY);
	//}

	//if (b.pos.z < -BOUNDARY) {
	//	f.z += (-BOUNDARY - b.pos.z);
	//}
	//else if (b.pos.z > BOUNDARY) {
	//	f.z += (b.pos.z - BOUNDARY);
	//}
	//return f * k_bd;
	if (b.pos.x < -BOUNDARY || b.pos.x > BOUNDARY) {
		b.v.x = -b.v.x * 0.8f;
	}

	if (b.pos.y < -BOUNDARY || b.pos.y > BOUNDARY) {
		b.v.y = -b.v.y * 0.8f;
	}

	if (b.pos.z < -BOUNDARY || b.pos.z > BOUNDARY) {
		b.v.z = -b.v.z * 0.8f;
	}

	return b.v;
}

int main(void) {
	// -------------- Setup the window and everything -------------- //
	io::GLFWContext windows;
	auto window =
		windows.create(io::Window::dimensions{ 960, 960 }, "A2 mass-spring system");
	window.enableVsync(true);

	auto view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	glClearColor(.5f, .5f, .5f, .5f);

	// -------------- Initialize the model -------------- //
	vector<Boid> boids;

	// -------------- Create renderable background -------------- //
	auto ground = PolyLine<givr::PrimitiveType::LINES>();
	for (int i = 0; i < 11; i++) {
		ground.push_back(Point(vec3(-25.f, 0.f, -25.f + i * 5.f)));
		ground.push_back(Point(vec3(25.f, 0.f, -25.f + i * 5.f)));
		ground.push_back(Point(vec3(-25.f + i * 5.f, 0.f, -25.f)));
		ground.push_back(Point(vec3(-25.f + i * 5.f, 0.f, 25.f)));
	}

	auto lineStyle1 = GL_Line(Colour(0.f, 0.f, 0.f));
	auto renderableGournd = givr::createRenderable(ground, lineStyle1);

	int t = 0;
	auto transform = scale(translate(givr::mat4f{ 1.f }, vec3(0, -0.f, 0)), givr::vec3f{ 1.f });
	auto phong = Phong(Colour(1.f, 1.f, 0.f), LightPosition(10.f, 10.f, 10.f));

	// --------------------- Temporary vars --------------------- //
	float r_avoid = 0.5;
	float r_align = 1.25;
	float r_cohesion = 2;

	window.run([&](float frameTime) {
		// Set the viewport
		view.projection.updateAspectRatio(window.width(), window.height());
		draw(renderableGournd, view, transform);

		// --------------------- Gradually add boids --------------------- //
		if (boids.size() < N_BOIDS) {
			Boid b;
			b.pos = ballRand(5.f);
			b.v = 10.f*ballRand(5.f);
			boids.push_back(b);

			b.pos = vec3(5.f, 0.f, 0.f) + ballRand(5.f);
			b.v = 10.f*ballRand(5.f);
			boids.push_back(b);
		}

		// --------------------- Update state --------------------- //
		// clear the forces
		for (int i = 0; i < boids.size(); i++) {
			boids[i].f = vec3(0.f);
		}
		// dumbest approach temporarily 
		for (int i = 0; i < boids.size(); i++) {
			for (int j = i-1; j > -1; j--) {
				float d = distance(boids[i].pos, boids[j].pos);
				if (d < r_avoid) {
					vec3 f = force_avoidance(d, boids[j].pos - boids[i].pos);
					boids[i].f -= f;
					boids[j].f += f;
				}
				else if (d < r_align) {
					vec3 f = force_alignment(d, boids[i].v, boids[j].v);
					boids[i].f += f;
					boids[j].f -= f;
				}
				else if (d < r_cohesion) {
					vec3 f = force_cohesion(d, boids[i].pos, boids[j].pos);
					boids[i].f += f;
					boids[j].f -= f;
				}
			}
		}
		// semi-implicit integration
		for (int i = 0; i < boids.size(); i++) {
			boids[i].v += boids[i].f * Dt;
			boids[i].v = force_boundary(boids[i]);
			boids[i].pos += boids[i].v * Dt;
		}


		// --------------------- Render --------------------- //
		for (auto b : boids) {
			auto sphere = Sphere(Centroid(b.pos), Radius(0.2));
			auto spheres = givr::createInstancedRenderable(sphere, phong);
			addInstance(spheres, transform);
			draw(spheres, view);
		}

		// --------------------- Render time --------------------- //
		t = t > 1000 ? 0 : (t+1);
	});

	exit(EXIT_SUCCESS);
}
