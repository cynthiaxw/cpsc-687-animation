//------------------------------------------------------------------------------
// A simple example showing how to use the triangle geometry
//------------------------------------------------------------------------------
#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

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
const float Dt= 0.02f;

typedef struct{
	float m;	// mass
	float w;	// 1/mass, w<=0 defines fixed particle
	vec3 pos;	// position
	vec3 v;		// velocity (with direction)
	vec3 F;		// net force
} Particle;

typedef struct {
	float len;	// constant desired rest length
	int i;				// particle index i
	int j;				// particle index j
	float ks;		// spring stiffness
	float kd;		// spring damping
} Spring;

typedef struct {
	vector<Particle> particles;
	vector<Spring> springs;
} Model;

Model initSingleSpring() {
	Model model;
	Particle p;
	p.m = 0.5;
	p.w = 0.f;
	p.F = vec3(0.f);
	p.v = vec3(0.f);
	p.pos = vec3(0.f, 35.f, 0.f);
	model.particles.push_back(p);
	p.w = 1 / p.m;
	p.pos -= vec3(0.f, 10.f, 0.f);
	model.particles.push_back(p);
	
	Spring s;
	s.i = 0; s.j = 1;
	s.ks = 5.5f;
	s.kd = 0.1f;
	s.len = 5;
	model.springs.push_back(s);

	return model;
}

Model initSpringChain() { 
	Model model;
	const int n_pts = 5;		// number of particles
	const float h = 35;			// height
	const float m = 0.5;		// mass of the particles, 2kg
	const float len = 3.0;		// length of the spring
	const float ks = 14.5;
	const float kd = 10.5;
	
	for (int i = 0; i < n_pts; i++) {
		Particle p;
		p.m = m;
		p.w = i == 0 ? 0 : (1.f / m);
		p.pos = vec3(i*len, h, 0);
		p.v = vec3(0.f);
		p.F = vec3(0.f);
		model.particles.push_back(p);
	}

	for (int i = 0; i < n_pts - 1; i++) {
		Spring s;
		s.len = len;
		s.i = i;
		s.j = i + 1;
		s.ks = ks;
		s.kd = kd; 
		model.springs.push_back(s);
	}
	return model;
}

Model initJello() {
	Model model;
	const int n_pts = 5;		// number of particles
	const float m = 0.5;		// mass of the particles, 2kg
	const float len = 3.0;		// length of the spring
	const float ks = 14.5;
	const float kd = 10.5;



	return model;
}

vec3 collisionForce(Particle p) {

	return vec3(0, 0, 0);
}

void updateNetForce(Model* model) {
	for (int i = 0; i < model->particles.size(); i++) {
		model->particles[i].F = vec3(0.f);
	}
	for (auto s : model->springs) {		// accumulate spring forces
		float dist = distance(model->particles[s.i].pos, model->particles[s.j].pos);
		// avoid division by a very small number
		dist = dist <= 0.000001 ? 0.000001 : dist;
		vec3 dir = (model->particles[s.i].pos - model->particles[s.j].pos) / dist;
		vec3 Fs = - s.ks * (dist - s.len) * (model->particles[s.i].pos - model->particles[s.j].pos) / dist;
		vec3 Fd = - s.kd * dot((model->particles[s.i].v - model->particles[s.j].v), dir) * dir;
		model->particles[s.i].F += Fs + Fd;
		model->particles[s.j].F -= Fs + Fd;
	}
	for (int i = 0; i < model->particles.size(); i++) {
		model->particles[i].F += collisionForce(model->particles[i]) + model->particles[i].m * g * vec3(0, -1, 0);
	}
}

void EulerIntegration(Model* model) {
	for (int i = 0; i < model->particles.size(); i++) {
		model->particles[i].v += model->particles[i].w * model->particles[i].F * Dt;
		model->particles[i].pos += model->particles[i].v * Dt;
	}
}


int main(void) {
	// -------------- Setup the window and everything -------------- //
	io::GLFWContext windows;
	auto window =
		windows.create(io::Window::dimensions{ 960, 960 }, "A2 mass-spring system");
	window.enableVsync(true);

	auto view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	glClearColor(1.f, 1.f, 1.f, 1.f);

	// -------------- Initialize the model -------------- //
	//Model springChain = initSpringChain();
	//Model* curmodel = &springChain;
	Model singleSpring = initSingleSpring();
	Model* curmodel = &singleSpring;
	int render_p = 1;

	// -------------- Create renderable particles -------------- //
	auto phong = Phong(Colour(1.f, 1.f, 0.f), LightPosition(10.f, 10.f, 10.f));


	// -------------- Create renderable ground -------------- //
	auto ground = PolyLine<givr::PrimitiveType::LINES>();
	for (int i = 0; i < 11; i++) {
		ground.push_back(Point(vec3(-25.f, 0.f, -25.f + i*5.f)));
		ground.push_back(Point(vec3(25.f, 0.f, -25.f + i * 5.f)));
		ground.push_back(Point(vec3(-25.f + i * 5.f, 0.f, -25.f)));
		ground.push_back(Point(vec3(-25.f + i * 5.f, 0.f, 25.f)));
	}
	//ground.push_back(Point(vec3(-25.f, 0.f, 0.f)));
	//ground.push_back(Point(vec3(25.f, 0.f, 0.f)));
	auto lineStyle1 = GL_Line(Colour(0.f, 0.f, 0.f));
	auto renderableGournd = givr::createRenderable(ground, lineStyle1);
	
	auto transform = scale(translate(givr::mat4f{ 1.f }, vec3(0, -15, 0)), givr::vec3f{ 1.f });

	float t = 0.f;

	window.run([&](float frameTime) {
		// Set the viewport
		view.projection.updateAspectRatio(window.width(), window.height());

		// --------------------- Update state --------------------- //
		updateNetForce(curmodel);
		EulerIntegration(curmodel);

		// -------------- Create renderable springs -------------- //
		auto springs = PolyLine<givr::PrimitiveType::LINES>();
		for (auto s : curmodel->springs) {
			springs.push_back(Point((curmodel->particles)[s.i].pos));
			springs.push_back(Point((curmodel->particles)[s.j].pos));
		}
		auto lineStyle = GL_Line(Colour(0.3, 0.3, 0.9));
		auto renderableLine = givr::createRenderable(springs, lineStyle);

		// --------------------- Render --------------------- //
		draw(renderableLine, view, transform);
		draw(renderableGournd, view, transform);

		if (render_p) {
			// add particle instances
			for (auto p : curmodel->particles) {
				auto sphere = Sphere(Centroid(p.pos), Radius(0.2));
				auto spheres = givr::createInstancedRenderable(sphere, phong);
				addInstance(spheres, transform);
				draw(spheres, view);
			}
		}

		// --------------------- Render time --------------------- //
		if (t > 1000.f) {
			t = 0.f;
		}
		else t += Dt;
	});

	exit(EXIT_SUCCESS);
}
