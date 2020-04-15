#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>

#include "io.h"
#include "turntable_controls.h"

#include <iostream>
#include <exception>
#include <cmath>
#include <vector>
#include <tuple>
#include <set>
#include <time.h>

using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using namespace std;

// Global Constants
const float g = 9.81f;
const float Dt = 0.1f;
const float Et = 10.f;	// estimate collision time
const float XBOUNDARY = 25.f;
const float YBOUNDARY = 20.f;
const float ZBOUNDARY = 25.f;
//const float XBOUNDARY = 75.f;
//const float YBOUNDARY = 60.f;
//const float ZBOUNDARY = 75.f;

// ============================ Data Structure ============================ //
typedef struct {
	vec3 pos;	// position
	//vec3 f = vec3(0.f);;		// net force
	vec3 v;		// velocity
	vec3 fs = vec3(0.f);		// separation
	vec3 fa = vec3(0.f);		// alignment
	vec3 fc = vec3(0.f);		// cohesion
	int cnt_s = 0;				// number of separation neighbors
	int cnt_a = 0;
	int cnt_c = 0;
} Boid;

typedef struct {
	vector<vec3> seed;
	vector<float> r;
	vector<float> speed;
	vector<vec2> obstacle_xz;		// x,z lcoation of the cylinder obstacle
	vector<float> obstacle_r;		// radius of the cylinder obstacle
}initState;

vector<set<int>> buckets;		// buckets assotiated with each cell
float h = 5.f;		// grid spacing
int g_nx = 12;		// number of cells
int g_ny = 10;
int g_nz = 12;
//int g_nx = 22;		// number of cells
//int g_ny = 18;
//int g_nz = 22;


// tunable parameters, can be read from text file
int N_BOIDS = 200;
float ks = 2.f;					// separation
float ka = 1.f;					// alignment
float kc = 1.f;					// cohesion

float k_bd = 6.f;				// boundary collision? should be steering
float rs = 1.f;
float ra = 2.5f;
float rc = 5.f;
const float RB = 1.f;


// global variables
float speedMag = 5.f;		// speed magnitude
float maxForce = 0.2f;		// max force

vec3 baitballCenter = vec3(0.f);
initState info;				// initial state read from text file

// ============================ Forces ======================= //
vec3 desiredForce(vec3 desired_v, vec3 v) {
	vec3 f;
	f = normalize(desired_v) * speedMag - v;
	float len = length(f);
	f = len > maxForce ? maxForce / len * f : f;
	return f;
}

vec3 boundaryForce(vec3 v, vec3 pos) {
	pos += v * Et;
	vec3 desired_v = v;
	int flg = 0;
	if (pos.x < -XBOUNDARY && v.x < 0 || pos.x > XBOUNDARY && v.x > 0) {
		desired_v.x = -desired_v.x;
		flg = 1;
	}

	if (pos.y < -YBOUNDARY && v.y < 0 || pos.y > YBOUNDARY && v.y > 0) {
		desired_v.y = -desired_v.y;
		flg = 1;
	}

	if (pos.z < -ZBOUNDARY && v.z < 0 || pos.z > ZBOUNDARY && v.z > 0) {
		desired_v.z = -desired_v.z;
		flg = 1;
	}
	if (!flg) return vec3(0.f);

	return k_bd*desiredForce(desired_v, v);
}

vec3 obstacleForce(vec2 v, vec2 pos) {
	vec3 f = vec3(0.f);
	for (int i = 0; i < info.obstacle_r.size(); i++) {
		float R = info.obstacle_r[i] + RB;
		vec2 xc = info.obstacle_xz[i] - pos;
		// check if the boid will be inside the obstacle ring after Et time
		if (length(xc) > R && distance(info.obstacle_xz[i], pos + v*1.5f) > R) {
			continue;
		};
		//cout << length(xc) << ", " << distance(info.obstacle_xz[i], pos + v * Et) << endl;
		// find v_perpendicular
		vec2 vp = normalize(v);
		vp *= dot(vp, xc);
		vp -= (xc);
		float len = length(vp);
		if (len < 0.0001) {
			f += R * normalize(cross(vec3(xc.x, 0.f, xc.y), vec3(0.f, 1.f, 0.f)));// / pow(length(xc), 2);
		}
		else {
			f += (R - length(vp)) * normalize(vec3(vp.x, 0.f, vp.y));// / pow(length(xc), 2);
		}
	}
	return 1.2f *f;
}

// ============================ Grid Operations ============================ //
tuple<int, int, int> getCellCoord(vec3 pos) {
	int x, y, z;
	if (pos.x < -XBOUNDARY) {
		x = 0;
	}
	else if (pos.x >= XBOUNDARY) {
		x = g_nx-1;
	}
	else {
		x = (int)((pos.x + XBOUNDARY) / h) + 1;
	}

	if (pos.y < -YBOUNDARY) {
		y = 0;
	}
	else if (pos.y >= YBOUNDARY) {
		y = g_ny - 1;
	}
	else {
		y = (int)((pos.y + YBOUNDARY) / h) + 1;
	}

	if (pos.z < -ZBOUNDARY) {
		z = 0;
	}
	else if (pos.z >= ZBOUNDARY) {
		z = g_nz - 1;
	}
	else {
		z = (int)((pos.z + ZBOUNDARY) / h) + 1;
	}

	return tuple(x, y, z);
}

int cellCoord2Id(tuple<int, int, int> coord) {
	return (g_nx * g_ny) * get<2>(coord) + g_nx * get<1>(coord) + get<0>(coord);
}

vector<int> getHalfNeighborCellId(int x, int y, int z) {
	vector<int> result;
	if (z > 0) {	// 3*3 z-1
		for (int i = x - 1; i < x + 2; i++) {
			if (i < 0 || i >= g_nx) { continue; }
			for (int j = y - 1; j < y + 2; j++) {
				if (j < 0 || j >= g_ny) { continue; }
				result.push_back(cellCoord2Id(tuple(i, j, z - 1)));
			}
		}
	}
	if (x < g_nx - 1) {		// 3 x+1
		for (int j = y - 1; j < y + 2; j++) {
			if (j < 0 || j >= g_ny) { continue; }
			result.push_back(cellCoord2Id(tuple(x-1, j, z)));
		}
	}
	if (y < g_ny - 1) {
		result.push_back(cellCoord2Id(tuple(x, y+1, z)));
	}

	return result;
}

// ============================ Update ============================ //
// for testing, not used
vec3 wraparound(vec3& p) {
	if (p.x > XBOUNDARY) {
		p.x = -XBOUNDARY;
	}
	else if (p.x < -XBOUNDARY) {
		p.x = XBOUNDARY;
	}
	if (p.y > YBOUNDARY) {
		p.y = -YBOUNDARY;
	}
	else if (p.y < -YBOUNDARY) {
		p.y = YBOUNDARY;
	}
	if (p.z > ZBOUNDARY) {
		p.z = -ZBOUNDARY;
	}
	else if (p.z < -ZBOUNDARY) {
		p.z = ZBOUNDARY;
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
	//for (int i = 0; i < boids->size(); i++) {
	//	for (int j = i - 1; j > -1; j--) {
	//		float d = distance(boids->at(i).pos, boids->at(j).pos);
	//		if (d < rs) {
	//			vec3 tmpf = (boids->at(i).pos - boids->at(j).pos) / (d * d);
	//			boids->at(i).fs += tmpf;
	//			boids->at(j).fs -= tmpf;
	//			boids->at(i).cnt_s++;
	//			boids->at(j).cnt_s++;
	//		}
	//		if (d < ra) {
	//			boids->at(i).fa += boids->at(j).v;		// temporarily store v
	//			boids->at(j).fa += boids->at(i).v;
	//			boids->at(i).cnt_a++;
	//			boids->at(j).cnt_a++;
	//		}
	//		if (d < rc) {
	//			boids->at(i).fc += boids->at(j).pos;
	//			boids->at(j).fc += boids->at(i).pos;
	//			boids->at(i).cnt_c++;
	//			boids->at(j).cnt_c++;
	//		}
	//	}
	//}

	// instead of using nested for loop, let use the buckets
	for (int i = 0; i < boids->size(); i++) {
		// find the cell location of boid i
		auto cellcoord = getCellCoord(boids->at(i).pos);
		int cell_id = cellCoord2Id(cellcoord);
		// check half of the 3*3*3 cube
		auto neighborBuckets = getHalfNeighborCellId(get<0>(cellcoord), get<1>(cellcoord), get<2>(cellcoord));
		neighborBuckets.push_back(cell_id);
		for (auto bc_id : neighborBuckets) {
			for (auto j : buckets[bc_id]) {	// iterate over all neighboring boids
				if (bc_id == cell_id && j >= i) {
					continue;
				}
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
}

void semiImplicitIntegration(vector<Boid>* boids) {
	vec3 ctmp = vec3(0.f);
	for (int i = 0; i < boids->size(); i++) {
		Boid b = boids->at(i);
		vec3 tmpf = vec3(0.f);
		vec3 tmpv = vec3(0.f);
		float len = 0;

		// separation force
		if (b.cnt_s > 0) {
			tmpv = b.fs / float(b.cnt_s);
			tmpf += ks * desiredForce(tmpv, b.v);
		}	

		// alignment force
		if (b.cnt_a > 0) {
			tmpv = b.fa / float(b.cnt_a);
			tmpf += ka * desiredForce(tmpv, b.v);
		}
		
		// cohesion force
		if (b.cnt_c > 0) {
			tmpv = b.fc / float(b.cnt_c);
			tmpf += kc * desiredForce(tmpv-b.pos, b.v);
		}
		
		// boundary force
		tmpf += boundaryForce(b.v, b.pos);

		//obstacle force
		if (model_n == 1 || model_n == 3) {
			tmpf += obstacleForce(vec2(b.v.x, b.v.z), vec2(b.pos.x, b.pos.z));
		}
		else if (model_n == 2 || model_n == 3) {	// bait ball
			tmpf += 2.5f * desiredForce(baitballCenter - b.pos, b.v);
		}

		// update velocity
		b.v += tmpf * Dt;

		// record old bucket id
		int cell_id_old = cellCoord2Id(getCellCoord(b.pos));

		// update position
		b.pos += b.v * Dt;
		ctmp += b.pos;
		//b.pos = wraparound(b.pos);

		// update buckets
		int cell_id_new = cellCoord2Id(getCellCoord(b.pos));
		if (cell_id_new != cell_id_old) {
			buckets[cell_id_old].erase(i);
			buckets[cell_id_new].insert(i);
		}

		// clear forces
		b.cnt_s = 0;
		b.cnt_a = 0;
		b.cnt_c = 0;
		b.fs = vec3(0.f);
		b.fa = vec3(0.f);
		b.fc = vec3(0.f);
		boids->at(i) = b;
	}
	baitballCenter = ctmp/(float)boids->size();
}

// ============================ File IO ============================ //
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

void drawbackground(PolyLine<givr::PrimitiveType::LINES>* ground) {
	for (int i = 0; i < 11; i++) {
		ground->push_back(Point(vec3(-25.f, -YBOUNDARY, -25.f + i * 5.f)));
		ground->push_back(Point(vec3(25.f, -YBOUNDARY, -25.f + i * 5.f)));
		ground->push_back(Point(vec3(-25.f + i * 5.f, -YBOUNDARY, -25.f)));
		ground->push_back(Point(vec3(-25.f + i * 5.f, -YBOUNDARY, 25.f)));
	}

	ground->push_back(Point(vec3(XBOUNDARY, YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, -YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, YBOUNDARY, -ZBOUNDARY)));

	ground->push_back(Point(vec3(-XBOUNDARY, YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, -YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, YBOUNDARY, -ZBOUNDARY)));

	ground->push_back(Point(vec3(XBOUNDARY, -YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, -YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, -YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, -YBOUNDARY, -ZBOUNDARY)));

	ground->push_back(Point(vec3(XBOUNDARY, YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, -YBOUNDARY, -ZBOUNDARY)));

	ground->push_back(Point(vec3(-XBOUNDARY, -YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, -YBOUNDARY, ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, -YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(XBOUNDARY, -YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, -YBOUNDARY, -ZBOUNDARY)));
	ground->push_back(Point(vec3(-XBOUNDARY, YBOUNDARY, -ZBOUNDARY)));


}
// ============================ Main ============================ //
int main(void) {
	// -------------- Setup the window and everything -------------- //
	io::GLFWContext windows;
	auto window =
		windows.create(io::Window::dimensions{ 1280, 960 }, "A2 mass-spring system");
	window.enableVsync(true);

	auto view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);

	glClearColor(.5f, .5f, .5f, .5f);

	// -------------- Read initial state from text file -------------- //
	info = readInitStateFromTextfile("../initState.txt");

	// -------------- Initialize the model -------------- //
	vector<Boid> boids;
	auto transMat = scale(translate(givr::mat4f{ 1.f }, vec3(0, -0.f, 0)), givr::vec3f{ 1.f });
	auto phong = Phong(Colour(1.f, 1.f, 0.f), LightPosition(30.f, 30.f, 30.f));
	auto fishGeo = givr::geometry::Mesh(Filename("../model/fish1.obj"));
	auto fishRenderable = givr::createInstancedRenderable(fishGeo, phong);

	auto obst_phong = Phong(Colour(0.474f, 0.658f, 0.823f), LightPosition(30.f, 30.f, 30.f));

	// -------------- Create renderable background -------------- //
	auto ground = PolyLine<givr::PrimitiveType::LINES>();
	drawbackground(& ground);

	auto lineStyle1 = GL_Line(Colour(0.f, 0.f, 0.f));
	auto renderableGournd = givr::createRenderable(ground, lineStyle1);

	// -------------- Initialize the grid -------------- //
	for (int i = 0; i < g_nx * g_ny * g_nz; i++) {
		buckets.push_back(set<int>());
	}
	int cnt_boid = 0;

	clock_t tt = clock();

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
		if (model_n == 1 || model_n == 3) {	// obstacle behaviour
			for (int i = 0; i < info.obstacle_r.size(); i++) {
				float x = info.obstacle_xz[i][0];
				float z = info.obstacle_xz[i][1];
				auto obst = Cylinder(Point1(x, -YBOUNDARY, z), Point2(x, YBOUNDARY, z), Radius(info.obstacle_r[i]));
				auto obstrenderable = givr::createInstancedRenderable(obst, obst_phong);
				addInstance(obstrenderable, transMat);
				draw(obstrenderable, view);
			}
		}	

		// --------------------- Gradually add boids --------------------- //
		if (cnt_boid < N_BOIDS-1) {
			for (int i = 0; i < info.r.size(); i++) {
				Boid b;
				b.pos = info.seed[i] + ballRand(info.r[i]);
				b.v = ballRand(info.speed[i]);
				boids.push_back(b);

				auto cellcoord = getCellCoord(b.pos);
				int cell_id = cellCoord2Id(cellcoord);
				buckets[cell_id].insert(cnt_boid);
				cnt_boid++;
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

		tt = clock() - tt;
		cout << "FPS: " << CLOCKS_PER_SEC / (float)tt << "\t\t #boids: " << cnt_boid+1 << "\r";
		tt = clock();
	});

	exit(EXIT_SUCCESS);
}
