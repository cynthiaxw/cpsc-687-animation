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
const float Dt = 0.005f;

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
	int showParticles = 0;
} Model;

void addSpring(Model* model, int i, int j, float len, float ks, float kd) {
	Spring s;
	s.i = i; s.j = j;
	s.len = len;
	s.ks = ks; s.kd = kd;
	model->springs.push_back(s);
}

void addParticle(Model* model, vec3 pos, float m, float w) {
	Particle p;
	p.m = m;
	p.w = w;
	p.F = vec3(0.f);
	p.v = vec3(0.f);
	p.pos = pos;
	model->particles.push_back(p);
}

Model initSingleSpring() {
	Model model;
	model.showParticles = 1;

	addParticle(&model, vec3(0.f, 35.f, 0.f), 0.5f, 0.f);
	addParticle(&model, vec3(0.f, 25.f, 0.f), 0.5f, 2.f);
	
	addSpring(&model, 0, 1, 5, 5.5, 0.1);

	return model;
}

Model initSpringChain() { 
	Model model;
	model.showParticles = 1;

	const int n_pts = 5;		// number of particles
	const float h = 35;			// height
	const float m = 0.5;		// mass of the particles, 2kg
	const float len = 3.0;		// length of the spring
	const float ks = 14.5;
	const float kd = 4.5;
	
	addParticle(&model, vec3(0, h, 0), m, 0.f);
	for (int i = 1; i < n_pts; i++) {	
		addParticle(&model, vec3(i * len, h, 0), m, 1.f/m);
	}

	for (int i = 0; i < n_pts - 1; i++) {
		addSpring(&model, i, i + 1, len, ks, kd);
	}
	return model;
}

Model initJello() {
	Model model;
	const float size = 10.f;
	const int n_pts = 6;				// number of particles per dimension
	const float h = size / (n_pts-1);	// spacing between particles
	const float m = 0.5f;				// mass
	const float ksee = 0.1f;			// damping factor
	const float ks = 25.0;
	const float kd = 2.f * ksee * sqrt(m * ks);

	// transformation matrix
	vec3 T = normalize(vec3(1, -1, 0.5));
	vec3 N = normalize(cross(vec3(0,0,1), T));
	vec3 B = normalize(cross(T, N));

	auto t = mat4(T[0], T[1], T[2], 0.f,
		N[0], N[1], N[2], 0.f,
		B[0], B[1], B[2], 0.f,
		-size/2.f, 30.f, -size/2.f, 1.f);

	for (int i = 0; i < n_pts; i++) {
		for (int j = 0; j < n_pts; j++) {
			for (int k = 0; k < n_pts; k++) {
				vec4 pos = t * vec4(k * h, j * h, i * h, 1.f);
				addParticle(&model, vec3(pos[0], pos[1], pos[2]), m, 1.f / m);
			}
		}
	}

	for (int i = 1; i < n_pts * n_pts * n_pts; i++) {
		for (int j = 0; j < i; j++) {
			float d = distance(model.particles[i].pos, model.particles[j].pos);
			if (d < 3.1 * h) {
				addSpring(&model, j, i, d, ks, kd);
			}
		}
	}
	return model;
}

Model initCloth(float size = 20.f, int n_pts = 41, vec3 x = vec3(1, 0, 0.7), vec3 leftup = vec3(-10, 35.0, 0)) {
	Model model;
	const float h = size / (n_pts - 1);	// spacing between particles
	const float m = 0.01f;				// mass
	const float ksee = 0.1f;			// damping factor
	const float ks = 100;
	const float kd = 0.1;//ks * Dt;//2.f * ksee * sqrt(m * ks);

	// transformation matrix
	vec3 T = normalize(x);
	vec3 N = normalize(cross(vec3(0, 0, 1), T));
	vec3 B = normalize(cross(T, N));

	auto t = mat4(T[0], T[1], T[2], 0.f,
		N[0], N[1], N[2], 0.f,
		B[0], B[1], B[2], 0.f,
		leftup[0], leftup[1], leftup[2], 1.f);

	
	for (int i = 0; i < n_pts; i++) {	//x
		for (int j = 0; j < n_pts; j++) {	//y
			vec4 pos = t * vec4(vec3(i * h, -j * h, -j * h), 1.f);
			if (j < 3 && (i % 20 < 1 || i % 20 > 18)){
				addParticle(&model, vec3(pos[0], pos[1], pos[2]), m, 0);
			}
			else {
				addParticle(&model, vec3(pos[0], pos[1], pos[2]), m, 1 / m);
			}
		}
	}

	for (int i = 1; i < n_pts * n_pts; i++) {
		for (int j = 0; j < i; j++) {
			float d = distance(model.particles[i].pos, model.particles[j].pos);
			if (d < 2.1 * h) {
				addSpring(&model, j, i, d, ks, kd);
			}
		}
	}

	return model;
}

// to model a square cloth falling on a table
Model initTableCloth(float size = 20.f, int n_pts = 41, vec3 x = vec3(1, 0, 0)) {
	Model model;
	const float h = size / (n_pts - 1);	// spacing between particles
	const float m = 0.01f;				// mass
	const float ksee = 0.1f;			// damping factor
	const float ks = 80;
	const float kd = 0.1;//ks * Dt;//2.f * ksee * sqrt(m * ks);
	vec3 leftup = vec3(-size/2.f, 25, size / 2.f);

	// transformation matrix
	vec3 T = normalize(x);
	vec3 N = vec3(0,0,1);
	vec3 B = normalize(cross(T, N));

	auto t = mat4(T[0], T[1], T[2], 0.f,
		N[0], N[1], N[2], 0.f,
		B[0], B[1], B[2], 0.f,
		leftup[0], leftup[1], leftup[2], 1.f);

	for (int i = 0; i < n_pts; i++) {	//x
		for (int j = 0; j < n_pts; j++) {	//y
			vec4 pos = t * vec4(vec3(i * h, -j * h, 0), 1.f);
			addParticle(&model, vec3(pos[0], pos[1], pos[2]), m, 1 / m);
		}
	}

	for (int i = 1; i < n_pts * n_pts; i++) {
		for (int j = 0; j < i; j++) {
			float d = distance(model.particles[i].pos, model.particles[j].pos);
			if (d < 2.1 * h) {
				addSpring(&model, j, i, d, ks, kd);
			}
		}
	}

	return model;
}

// Not used
vec3 collisionForce(Particle p) {
	float kc = 30.0f;
	if (p.pos.y < 0) {
		return - kc * p.pos.y * vec3(0, 1, 0);
	}
	return vec3(0, 0, 0);
}

void updateNetForce(Model* model) {
	for (int i = 0; i < model->particles.size(); i++) {
		model->particles[i].F = model->particles[i].m * g * vec3(0, -1, 0);
	}
	for (auto s : model->springs) {		// accumulate spring forces
		float dist = distance(model->particles[s.i].pos, model->particles[s.j].pos);
		// avoid division by a very small number
		dist = dist <= 0.0001 ? 0.0001 : dist;
		vec3 dir = (model->particles[s.i].pos - model->particles[s.j].pos) / dist;
		vec3 Fs = - s.ks * (dist - s.len) * (model->particles[s.i].pos - model->particles[s.j].pos) / dist;
		vec3 Fd = - s.kd * dot((model->particles[s.i].v - model->particles[s.j].v), dir) * dir;
		model->particles[s.i].F += Fs + Fd;
		model->particles[s.j].F -= Fs + Fd;
	}
}

void EulerIntegration(Model* model, float dt) {
	float cr = 0.6;
	for (int i = 0; i < model->particles.size(); i++) {
		model->particles[i].v += model->particles[i].w * model->particles[i].F * dt;
		float ypos = model->particles[i].pos.y + model->particles[i].v.y * dt;
		if (ypos < 0 && model->particles[i].v.y<0) {
			model->particles[i].v.y = -cr * model->particles[i].v.y;
		}
		model->particles[i].pos += model->particles[i].v * dt;
	}
}


const float TABLE_R = 5.2f;
const float TABLE_H = 10.f;

int onTableTop(vec3 pos, float R, float H) {
	if (pos.x * pos.x + pos.z * pos.z < R* R && pos.y < H && pos.y > H - 1) {
		return 1;
	}
	return 0;
}

void EulerIntegrationTableCloth(Model* model, float dt) {
	for (int i = 0; i < model->particles.size(); i++) {
		model->particles[i].v += model->particles[i].w * model->particles[i].F * dt;
		// check collision with the table top
		vec3 tmppos = model->particles[i].pos + model->particles[i].v * dt;
		if (onTableTop(tmppos, TABLE_R, TABLE_H)) {
				model->particles[i].v = vec3(0.f);
		}
		model->particles[i].pos += model->particles[i].v * dt;
	}
	

}

vector<Model> models;
Model* curmodel;


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
	models.push_back(initSingleSpring());
	models.push_back(initSpringChain());
	models.push_back(initJello());
	models.push_back(initCloth());
	models.push_back(initTableCloth());

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

	auto lineStyle1 = GL_Line(Colour(0.f, 0.f, 0.f));
	auto renderableGournd = givr::createRenderable(ground, lineStyle1);
	
	auto transform = scale(translate(givr::mat4f{ 1.f }, vec3(0, -15, 0)), givr::vec3f{ 1.f });
	auto table_t = scale(translate(givr::mat4f{ 1.f }, vec3(0, -15, 0)), givr::vec3f{ 5.f });

	float t = 0.f;

	// -------------- Create renderable table -------------- //
	auto table = givr::geometry::Mesh(Filename("../table/table1.obj"));
	auto renderableTable = givr::createInstancedRenderable(table, phong);

	window.run([&](float frameTime) {
		// Set the viewport
		view.projection.updateAspectRatio(window.width(), window.height());

		curmodel = &(models[model_n]);
		int render_p = curmodel->showParticles;
		if (reset) {
			// reset current model
			switch (model_n) {
			case 0:
				*curmodel = initSingleSpring(); break;
			case 1:
				*curmodel = initSpringChain(); break;
			case 2:
				*curmodel = initJello(); break;
			case 3:
				*curmodel = initCloth(); break;
			case 4:
				*curmodel = initTableCloth(); break;
			}
			reset = 0;
		}


		// --------------------- Update state --------------------- //
		for (int i = 0; i < 8; i++) {
			updateNetForce(curmodel);
			if (model_n == 4) {
				EulerIntegrationTableCloth(curmodel, Dt);
			}
			else {
				EulerIntegration(curmodel, Dt);
			}
		}

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

		if (model_n == 4) {
			addInstance(renderableTable, table_t);
			draw(renderableTable, view);
		}

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
