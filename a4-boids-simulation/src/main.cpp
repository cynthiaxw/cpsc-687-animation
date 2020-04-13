#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>

#include "io.h"
#include "turntable_controls.h"

#include <iostream>
#include <exception>


using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using namespace std;

// Global Constants
const float g = 9.81f;
const float Dt = 0.1f;
const float BOUNDARY = 25.f;

// tunable parameters, can be read from text file
int N_BOIDS = 200;
float ks = 2.f;			// separation
float ka = 1.f;			// alignment
float kc = 1.f;					// cohesion

float k_bd = 0.9f;				// boundary collision? should be steering
float rs = 2;
float ra = 5;
float rc = 10;

float speedMag = 5.f;		// speed magnitude
float maxForce = 0.2f;		// max force

typedef struct {
	vec3 pos;	// position
	//vec3 f = vec3(0.f);;		// net force
	vec3 v = vec3(0.f);;		// velocity
	vec3 fs = vec3(0.f);;		// separation
	vec3 fa = vec3(0.f);;		// alignment
	vec3 fc = vec3(0.f);		// cohesion
	int cnt_s = 0;				// number of separation neighbors
	int cnt_a = 0;
	int cnt_c = 0;
} Boid;

vec3 force_boundary(Boid b) {
	if (b.pos.x < -BOUNDARY || b.pos.x > BOUNDARY) {
		b.v.x = -b.v.x * k_bd;
	}

	if (b.pos.y < -BOUNDARY || b.pos.y > BOUNDARY) {
		b.v.y = -b.v.y * k_bd;
	}

	if (b.pos.z < -BOUNDARY || b.pos.z > BOUNDARY) {
		b.v.z = -b.v.z * k_bd;
	}

	return b.v;
}

// for testing, not used
vec3 wraparound(vec3& p) {
	if (p.x > BOUNDARY) {
		p.x = -BOUNDARY;
	}
	else if (p.x < -BOUNDARY) {
		p.x = BOUNDARY;
	}
	if (p.y > BOUNDARY) {
		p.y = -BOUNDARY;
	}
	else if (p.y < -BOUNDARY) {
		p.y = BOUNDARY;
	}
	if (p.z > BOUNDARY) {
		p.z = -BOUNDARY;
	}
	else if (p.z < -BOUNDARY) {
		p.z = BOUNDARY;
	}
	return p;
}

mat4 boidTransformMat(Boid& b) {
	vec3 T = normalize(b.v);
	//vec3 N = normalize(b.f);
	vec3 N = vec3(0.f, 1.f, 0.f);
	vec3 B = normalize(cross(T, N));
	N = normalize(cross(B, T));

	mat4 m = mat4(T[0], T[1], T[2], 0.f,
		N[0], N[1], N[2], 0.f,
		B[0], B[1], B[2], 0.f,
		b.pos[0], b.pos[1], b.pos[2], 1.f);

	m = scale(m, vec3(1.5f));

	return m;
}

void updateNetForce(vector<Boid>* boids) {
	for (int i = 0; i < boids->size(); i++) {
		for (int j = i - 1; j > -1; j--) {
			float d = distance(boids->at(i).pos, boids->at(j).pos);
			if (d < rs) {
				vec3 tmpf = (boids->at(i).pos - boids->at(j).pos) / (d * d);
				boids->at(i).fs += tmpf;
				boids->at(j).fs -= tmpf;
				boids->at(i).cnt_s++;
				boids->at(j).cnt_s++;
			}
			if (d < ra) {
				boids->at(i).fa += boids->at(j).v;		// temporarily store v
				boids->at(j).fa += boids->at(i).v;
				boids->at(i).cnt_a++;
				boids->at(j).cnt_a++;
			}
			if (d < rc) {
				boids->at(i).fc += boids->at(j).pos;
				boids->at(j).fc += boids->at(i).pos;
				boids->at(i).cnt_c++;
				boids->at(j).cnt_c++;
			}
		}
	}
}

void semiImplicitIntegration(vector<Boid>* boids) {
	for (int i = 0; i < boids->size(); i++) {
		Boid b = boids->at(i);
		vec3 tmpf = vec3(0.f);
		vec3 tmpv = vec3(0.f);
		float len = 0;

		// separation force
		if (b.cnt_s > 0) {
			tmpv = b.fs / float(b.cnt_s);
			tmpv = normalize(tmpv) * speedMag - b.v;
			len = length(tmpv);
			tmpf += len > maxForce ? (ks * maxForce / len * tmpv) : (ks * tmpv);
		}		
			
		// alignment force
		if (b.cnt_a > 0) {
			tmpv = b.fa / float(b.cnt_a);
			tmpv = normalize(tmpv) * speedMag - b.v;
			len = length(tmpv);
			tmpf += len > maxForce ? ka * maxForce / len * tmpv : ka * tmpv;
		}
		
		// cohesion force
		if (b.cnt_c > 0) {
			tmpv = b.fc / float(b.cnt_c);
			tmpv = normalize(tmpv - b.pos) * speedMag - b.v;
			len = length(tmpv);
			tmpf += len > maxForce ? (kc * maxForce / len * tmpv) : (kc * tmpv);
		}
		
		// update velocity
		b.v += tmpf * Dt;

		// update position
		b.pos += b.v * Dt;
		b.pos = wraparound(b.pos);

		// clear forces
		b.cnt_s = 0;
		b.cnt_a = 0;
		b.cnt_c = 0;
		b.fs = vec3(0.f);
		b.fa = vec3(0.f);
		b.fc = vec3(0.f);
		boids->at(i) = b;
	}
}

// ============================ File IO ============================ //
typedef struct {
	vector<vec3> seed;
	vector<float> r;
	vector<float> speed;
	vector<vec2> obstacle_xz;		// x,z lcoation of the cylinder obstacle
	vector<float> obstacle_r;		// radius of the cylinder obstacle
}initState;

initState readInitStateFromTextfile(string const &path) {
	using std::istreambuf_iterator;
	using std::string;
	using std::stringstream;
	initState info;
	try {
		std::ifstream file(path);
		string line;
		size_t index;
		while (getline(file, line)) {	// read the file line by line
			// remove comments
			index = line.find_first_of("#");
			if (index != string::npos) {
				line.erase(index, string::npos);
			}
			// removes leading junk
			line.erase(0, line.find_first_not_of(" \t\r\n\v\f"));
			if (line.empty()) {
				continue; // empty or commented out line
			}
			std::istringstream in(line);
			string type;
			in >> type;
			if (type == "nboid") {
				in >> N_BOIDS;
			}
			if (type == "ks") {
				in >> ks;
			}
			else if (type == "ka") {
				in >> ka;
			}
			else if (type == "kc") {
				in >> kc;
			}
			else if (type == "boid") {
				float x, y, z, r, v;
				in >> x >> y >> z >> r >> v;
				info.seed.push_back(vec3(x, y, z));
				info.r.push_back(r);
				info.speed.push_back(v);
			}
			else if (type == "obstacle") {
				float x, z, r;
				in >> x >> z >> r;
				info.obstacle_xz.push_back(vec2(x, z));
				info.obstacle_r.push_back(r);
			}
		}
	}
	catch (exception & e) {
		cout << e.what() << endl;
		info.seed.push_back(vec3(0.f));
		info.r.push_back(5.f);
		info.speed.push_back(3.f);
	}
	return info;
}

// ============================ Main ============================ //
int main(void) {
	// -------------- Setup the window and everything -------------- //
	io::GLFWContext windows;
	auto window =
		windows.create(io::Window::dimensions{ 960, 960 }, "A2 mass-spring system");
	window.enableVsync(true);

	auto view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	glClearColor(.5f, .5f, .5f, .5f);

	// -------------- Read initial state from text file -------------- //
	initState info = readInitStateFromTextfile("../initState.txt");

	// -------------- Initialize the model -------------- //
	vector<Boid> boids;
	auto transMat = scale(translate(givr::mat4f{ 1.f }, vec3(0, -0.f, 0)), givr::vec3f{ 1.f });
	auto phong = Phong(Colour(1.f, 1.f, 0.f), LightPosition(30.f, 30.f, 30.f));
	auto fishGeo = givr::geometry::Mesh(Filename("../model/fish1.obj"));
	auto fishRenderable = givr::createInstancedRenderable(fishGeo, phong);

	auto obst_phong = Phong(Colour(0.474f, 0.658f, 0.823f), LightPosition(30.f, 30.f, 30.f));

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

	window.run([&](float frameTime) {
		// Set the viewport
		view.projection.updateAspectRatio(window.width(), window.height());

		// --------------------- Check key controls --------------------- //
		if (reset) {
			boids.clear();
			reset = 0;
		}

		// --------------------- Draw background --------------------- //
		draw(renderableGournd, view, transMat);
		for (int i = 0; i < info.obstacle_r.size(); i++) {
			float x = info.obstacle_xz[i][0];
			float z = info.obstacle_xz[i][1];
			auto obst = Cylinder(Point1(x, -BOUNDARY, z), Point2(x, BOUNDARY, z), Radius(info.obstacle_r[i]));
			auto obstrenderable = givr::createInstancedRenderable(obst, obst_phong);
			addInstance(obstrenderable, transMat);
			draw(obstrenderable, view);
		}

		// --------------------- Gradually add boids --------------------- //
		if (boids.size() < N_BOIDS) {
			for (int i = 0; i < info.r.size(); i++) {
				Boid b;
				b.pos = info.seed[i] + ballRand(info.r[i]);
				b.v = ballRand(info.speed[i]);
				boids.push_back(b);
			}
		}

		// --------------------- Update state --------------------- //
		updateNetForce(&boids);
		semiImplicitIntegration(&boids);

		// --------------------- Render --------------------- //
		for (auto b : boids) {
			auto t = boidTransformMat(b);
			addInstance(fishRenderable, t);
			draw(fishRenderable, view);
		}

		// --------------------- Timer --------------------- //
		t = t > 1000 ? 0 : (t+1);
	});

	exit(EXIT_SUCCESS);
}
