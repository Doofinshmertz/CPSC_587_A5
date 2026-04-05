#pragma once

#include <vector>
#include <givr.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/transform.hpp>

#include <thread>
#include <atomic>

#include <unordered_map>

#define SPHERE_MAXR 20.0f
#define SPHERE_MINR 6.0f
#define NUM_SPHERES 24

#define SIMULATION_THREADS 16
#define RENDER_THREADS 8

enum ColliderType
{
	C_Plane,
	C_Sphere
};
//Boids in simulation
struct Boid {
	glm::vec3 p = glm::vec3(0.f);
	glm::vec3 v = glm::vec3(0.f);
	size_t cell_index = 0;
	bool isValid() const;
	//Note: Mass is implicitly 1 (our choice) so Force = Acceleration
	//TO-DO: Modify this class to include certain desired quantities (mass, force, ...)
	//May even add functions! Such as integration ...
};

struct Collider
{
	ColliderType type = ColliderType::C_Plane; 
};

// Walls for avoidences and simulation boundry
struct Plane : public Collider
{
	bool isValid() const;
	// TO-DO: Define a plane for collision avoidence and container purposes

	glm::vec3 point;
	glm::vec3 normal;
};

// Spheres for avoidences
struct Sphere : public Collider
{
	bool isValid() const;
	// TO-DO: Define a sphere for collision avoidence purposes
	glm::vec3 point;

	float radius;
};

bool allValid(const std::vector<Boid>& boids);
bool allValid(const std::vector<Plane>& planes);
bool allValid(const std::vector<Sphere>& spheres);

// If you want to use a different view, change this and the one in main
using ModelViewContext = givr::camera::ViewContext<
	givr::camera::TurnTableCamera,
	givr::camera::PerspectiveProjection>;

// the information needed by the thread for rendering task
struct RenderTask
{
	size_t start_index;
	size_t end_index;
};

struct CalculationTask
{
	size_t start_index;
	size_t end_index;
};

class BoidSimulation
{
private:
	// the number of boids 
	size_t num_boids;
	// thee arrays, two for rendering, one for updateing
	std::vector<Boid> boids_a;
	std::vector<Boid> boids_b;
	std::vector<Boid> boids_c;
	
	// planes and spheres
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;

	// the pointers to the two arrays to use for rendering
	std::atomic<std::vector<Boid>*> start_positions = nullptr;
	std::atomic<std::vector<Boid>*> end_positions = nullptr;
	// the pointer to the array that is to be written to
	std::atomic<std::vector<Boid>*> to_positions = nullptr;

	// detection angles
	float angle_sep;
	float angle_align;
	float angle_coh;

	// detection radius
	float r_sep;
	float r_align;
	float r_coh;

	// acceleration factors
	float k_sep;
	float k_align;
	float k_coh;

	// offset to apply to surfaces
	float offset;
	// repulsion force from surfaces
	float k_repulsion;

	// the acceleration and speed budget for the boids
	float max_speed;
	float max_acc; 

	// maximum detection radius (needed for some bounding tasks)
	float max_r;
	float max_r_inv;
	// the x, y, z length of the simulation area (the extents will be in both directions (+bound_x to -bound_x)
	float bound_x = 100.0f;
	float bound_y = 100.0f;
	float bound_z = 100.0f;

	// the number of bins in the x,y,z directions
	size_t bins_x;
	size_t bins_y;
	size_t bins_z;
	size_t num_per_bin;
	
	// use for array indexing
	size_t x_jump; // value needed to jump forward by one bin in the x direction
	size_t y_jump; // value needed to jump forward by one bin in the y direction
	size_t z_jump; // value needed to jump forward by one bin in the z direction

	// the factors to multipy x and y by to get the correct cell index
	size_t x_fact;
	size_t y_fact;

	// the location of one all negative corner of the uniform grid 
	float origin_x;
	float origin_y;
	float origin_z;

	// simulation controlls
	bool new_buffers = false;
	// the time elapsed since the last buffer swap
	float elapsed_time = 0.0f;
	std::atomic<float> delta_time = 0.0f; // the time difference between the last two simulation steps
	// if the simulation should be paused
	std::atomic<bool> is_paused = true;
	std::atomic<bool> stop_simulation = true;

	// Simulation Threads
	std::thread threads_sim[SIMULATION_THREADS];
	CalculationTask sim_tasks[SIMULATION_THREADS];
	// the thread for the main simulation loop
	std::thread main_loop_thread;

	// the uniform grid (flattened, indexed by x*(bins_y*bins_z*num_per_bin) + y*bins_z*num_per_bin + z * num_per_bin + <index_in_bin> + 1 (the first element in each bin stores how full the bin is)
	std::vector<uint32_t> uniform_grid;
	size_t num_bins;

	std::unordered_map<size_t, std::vector<Collider*>> collider_map;
	// rendering stuff
	givr::geometry::Mesh boid_geometry;
	givr::style::Phong boid_style;
	givr::InstancedRenderContext<givr::geometry::Mesh, givr::style::Phong> boid_render[RENDER_THREADS];
	RenderTask rendering_tasks[RENDER_THREADS]; // each thread gets a task
	// rendering threads
	std::thread threads_rnd[RENDER_THREADS];


	givr::geometry::MultiLine wall_geometry;
	givr::style::LineStyle wall_style;
	givr::RenderContext<givr::geometry::MultiLine, givr::style::LineStyle> wall_render;

	givr::geometry::Sphere sphere_geometry;
	givr::style::Phong sphere_style;
	givr::InstancedRenderContext<givr::geometry::Sphere, givr::style::Phong> sphere_render;

public:
	// constructor
	BoidSimulation();
	
	// set the simulation parameters
	void SetSimulationParameters(float _angle_sep, float _angle_align, float _angle_coh,
		float _r_sep, float _r_align, float _r_coh,
		float _k_sep, float _k_align, float _k_coh,
		float _offset, float _k_repulsion, 
		float _max_speed, float _max_acceleration, 
		size_t _num_boids);

	/**
	 * stop the current thread, then reset the positions of the boids
	 */
	void reset();
	void pauseSimulation();
	void playSimulation();
	void step(float dt);

	// reder the current frame
	void render(const ModelViewContext &view);
	bool isValid() const;

	float GetDeltatime();

	~BoidSimulation();

private:
	// setup the simulation
	void SetupSimulation();
	// give the boids random starting positions
	void SetupBoids();
	// setup walls
	void SetupWalls();
	// setup spheres
	void SetupSpheres();
	// pre-sort the planes and sphers into a unordered map
	void PlaceCollidersOnGrid();
	// reset the positions of the boids
	void ResetBoidPositions();
	// sort into bins
	void SortIntoGrid();

	// the simulation loop:
	void SimulationLoop();

	// the simulation function that runs in a thread
	void SimulateBoids(size_t thread_id);

	// the rendering function
	void AddBoidRenderable(size_t thread_id);

};

//Helper
namespace glm {
	bool isallfinite(glm::vec3 v);
}

namespace simulation {
	namespace primatives {


	} // namespace primatives

	namespace models {
		//If you want to use a different view, change this and the one in main
		using ModelViewContext = givr::camera::ViewContext<
			givr::camera::TurnTableCamera, 
			givr::camera::PerspectiveProjection
		>;
		// Abstract class used by all models
		class GenericModel {
		public:
			virtual void reset() = 0;
			virtual void step(float dt) = 0;
			virtual void render(const ModelViewContext& view) = 0;
			virtual bool isValid() const { return true; } // Don't HAVE to define
		};

		//Model for simulation
		class BoidsModel : public GenericModel {
		public:
			BoidsModel();
			void reset();
			void step(float dt);
			void render(const ModelViewContext& view);
			bool isValid() const;

			//Simulation Constants (you can re-assign values here from imgui)
			glm::vec3 g = { 0.f, -9.81f, 0.f };
			size_t n_boids = 100; //need alot more eventually for full assignment

		private:
			//Simulation Parts
			std::vector<Boid> boids;
			//std::vector<primatives::plane> planes;
			//std::vector<primatives::sphere> spheres;

			//Render
			givr::geometry::Mesh boid_geometry;
			givr::style::Phong boid_style;
			givr::InstancedRenderContext<givr::geometry::Mesh, givr::style::Phong> boid_render;

			givr::geometry::MultiLine wall_geometry;
			givr::style::LineStyle wall_style;
			givr::RenderContext<givr::geometry::MultiLine, givr::style::LineStyle> wall_render;

			//givr::geometry::TriangleSoup wall_geometry;
			//givr::style::Phong wall_style;
			// Maybe changed this (below) to instanced render????????
			//givr::RenderContext<givr::geometry::TriangleSoup, givr::style::Phong> wall_render;

			givr::geometry::Sphere sphere_geometry;
			givr::style::Phong sphere_style;
			givr::InstancedRenderContext<givr::geometry::Sphere, givr::style::Phong> sphere_render;
		};

	} // namespace models
} // namespace simulation