#include "models.hpp"
#include <random>
#include <stdio.h>
#include <chrono>

// constructor
BoidSimulation::BoidSimulation()
	: boid_geometry(givr::geometry::Mesh(givr::geometry::Filename("./models/dart.obj"))),
	boid_style(givr::style::Colour(1.f, 1.f, 0.f), givr::style::LightPosition(100.f, 100.f, 100.f)),
	wall_geometry(), wall_style(givr::style::Colour(0.f, 1.0f, 0.0f)),
	sphere_geometry(givr::geometry::Radius(1.0f)),
	sphere_style(givr::style::Colour(1.0f, 0.0f, 1.0f), givr::style::LightPosition(100.0f, 100.0f, 100.0f))
{
	new_buffers = true;
	elapsed_time = 0.0f;

	wall_render = givr::createRenderable(wall_geometry, wall_style);
	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		boid_render[i] = givr::createInstancedRenderable(boid_geometry, boid_style);
	}
	sphere_render = givr::createInstancedRenderable(sphere_geometry, sphere_style);
}

BoidSimulation::~BoidSimulation()
{
	// we have to make sure the thread stoped
	stop_simulation = true;
	if(main_loop_thread.joinable())
	{
		main_loop_thread.join();
	}
}

// set the simulation parameters
void BoidSimulation::SetSimulationParameters(float _angle_sep, float _angle_align, float _angle_coh,
							 float _r_sep, float _r_align, float _r_coh,
							 float _k_sep, float _k_align, float _k_coh,
							 float _offset, float _k_repulsion,
							 float _max_speed, float _max_acceleration,
							 size_t _num_boids)
{
	// set all values
	angle_sep = _angle_sep;
	angle_align = _angle_align;
	angle_coh = _angle_coh;
	r_sep = _r_sep;
	r_align = _r_align;
	r_coh = _r_coh;
	k_sep = _k_sep;
	k_align = _k_align;
	k_coh = _k_coh;
	offset = _offset;
	k_repulsion = _k_repulsion;
	max_speed = _max_speed;
	max_acc = _max_acceleration;
	num_boids = _num_boids;

	// need to re-calculate grid spacing
	// re-initialze the boids

	// if a simulation was already running, restart it, otherwise do not
	if((stop_simulation == false) || main_loop_thread.joinable())
	{
		// set stop simulation to true
		stop_simulation = true;

		main_loop_thread.join();
		SetupSimulation();
		stop_simulation = false;

		// start the simulation loop
		main_loop_thread = std::thread(&BoidSimulation::SimulationLoop, this);
	}
	else
	{
		printf("starting simulation\n");
		// reset the boid positions
		SetupSimulation();
		// start a new simulation
		stop_simulation = false;
		// start the simulation loop
		main_loop_thread = std::thread(&BoidSimulation::SimulationLoop, this);
	}
}

/**
 * stop the current thread, then reset the positions of the boids
 */
void BoidSimulation::reset()
{
	// set stop simulation to true
	stop_simulation = true;

	main_loop_thread.join();
	ResetBoidPositions();
	stop_simulation = false;

	// start the simulation loop
	main_loop_thread = std::thread(&BoidSimulation::SimulationLoop, this);
}

void BoidSimulation::pauseSimulation()
{
	is_paused = true;
}

void BoidSimulation::playSimulation()
{
	is_paused = false;
}

void BoidSimulation::step(float dt)
{
	elapsed_time += dt;
}

void BoidSimulation::SimulationLoop()
{

	printf("simulation started\n");
	// setup time measurement variables
	std::chrono::steady_clock::time_point start_t = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end_t = start_t;
	delta_time = 0.01f; // starting delta time, we don't what this to be equal to zero
	
	float del = 0.0f;
	// while stop simulation is not true, 
	while(!stop_simulation)
	{
		// if the simulation is paused, sleep for some time, then try again
		if(is_paused)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			continue;
		}

		SortIntoGrid();

		// dispatch threads
		// join threads

		// swap buffers
		std::vector<Boid>* temp = end_positions;
		
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		// measure the time step
		end_t = std::chrono::steady_clock::now();
		delta_time = (std::chrono::duration<float, std::ratio<1>>(end_t - start_t)).count();
		start_t = end_t;

		del = delta_time;


	}

	printf("simulation stopped\n");
}

void BoidSimulation::SimulateBoids(size_t thread_id)
{
	size_t start = sim_tasks[thread_id].start_index;
	size_t end = sim_tasks[thread_id].end_index;

	for(size_t i = start; i < end; i++)
	{
		// first compute wall
		// iterate over potential neighbors
		// if is neighbor, then calculate appropriate forces
		
	}
}

float BoidSimulation::GetDeltatime()
{
	return delta_time;
}

void BoidSimulation::render(const ModelViewContext &view)
{
	// render spheres
	for(size_t i = 0; i < NUM_SPHERES; i++)
	{
		givr::addInstance(sphere_render, glm::scale( glm::translate(glm::mat4(1.0f), spheres[i].point), glm::vec3(spheres[i].radius, spheres[i].radius, spheres[i].radius)));
	}

	// dispatch render threads
	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		threads_rnd[i] = std::thread(&BoidSimulation::AddBoidRenderable, this, i);
	}

	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		threads_rnd[i].join();
	}

	// draw
	givr::style::draw(wall_render, view);
	givr::style::draw(sphere_render, view);
	
	// draw all boids
	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		givr::style::draw(boid_render[i], view);
	}

	ResetBoidPositions();
}

void BoidSimulation::AddBoidRenderable(size_t thread_id)
{
	size_t start = rendering_tasks[thread_id].start_index;
	size_t end = rendering_tasks[thread_id].end_index;

	if(start == end){ return;}
	// render the boids in between the given indices
	for(size_t i = start; i < end; i++)
	{
		givr::addInstance(boid_render[thread_id], glm::translate(glm::mat4(1.0f), boids_a[i].p));
	}
}

void BoidSimulation::SetupSimulation()
{
	SetupBoids();
	SetupWalls();
	SetupSpheres();

	// the collider grid only has to be created once
	PlaceCollidersOnGrid();
	// the boids have to be pre-sorted before the first iteration,
	// then this no-longer needs to be called
	SortIntoGrid();
}

void BoidSimulation::SetupBoids()
{
	// initialize arrays for holding the boids
	boids_a.resize(num_boids);
	boids_b.resize(num_boids);
	boids_c.resize(num_boids);

	// set the arrays for rendering and simulation
	start_positions = &boids_a;
	end_positions = &boids_b;
	to_positions = &boids_c;

	// starting configuration for the renderer
	new_buffers = true;
	elapsed_time = 0.0f;

	// need to setup the uniform grid 
	// calculate the number of bins in each direction

	// get the largest detection radius
	max_r = std::max(std::max(r_sep, r_align), r_coh);
	max_r_inv = 1.0f / max_r;
	// calculate the number of bins in the x,y,z direction (+1 incase the bounds not evenlly divisible by the radius)
	bins_x = 2 * (size_t(bound_x / max_r) + 1);
	bins_y = 2 * (size_t(bound_y / max_r) + 1);
	bins_z = 2 * (size_t(bound_z / max_r) + 1);

	// calculate the location of the all negative corner of the uniform grid (this will be used as the origin)
	origin_x = -float((bins_x >> 1)) * max_r;
	origin_y = -float((bins_y >> 1)) * max_r;
	origin_z = -float((bins_z >> 1)) * max_r;

	// approximate the number per bin as the average number of boids per cell in the simulation multiplied by 2 plus 10 (x*2 ensures ratio of at least 2, +10 accounts for sparse grid scenario where num_boids/bins is near zero) 
	num_per_bin = ((num_boids / (bins_x * bins_y * bins_z)) + 1)*2 + 1 + 10;

	// the jumps (values needed to jump the grid by one bin)
	x_jump = bins_y * bins_z * num_per_bin;
	y_jump = bins_z*num_per_bin;
	z_jump = num_per_bin;

	// used for finding cell indices
	x_fact = bins_y * bins_z;
	y_fact = bins_z;

	num_bins = x_jump*bins_x;
	// initialize the uniform grid
	uniform_grid.resize(num_bins);

	// setup the boid rendering tasks
	// first get the number of boids per task

	size_t num_per_task;
	if(num_boids < RENDER_THREADS)
	{
		// for such a small amount of boids, we can just use one thread
		num_per_task = num_boids;
		rendering_tasks[0].start_index = 0;
		rendering_tasks[0].end_index = num_boids;
		printf("\n\nboids less than threads, start index: %ld, end index: %ld", 0, num_boids);
		// do not use the other tasks
		for(size_t i = 1; i < RENDER_THREADS; i++)
		{
			rendering_tasks[i].start_index = 0;
			rendering_tasks[i].end_index = 0;
		}
	}
	else
	{
		printf("\n\nThread distribution:\n");
		size_t num_per_task = num_boids / RENDER_THREADS;
		for(size_t i = 0; i < RENDER_THREADS; i++)
		{
			rendering_tasks[i].start_index = i * num_per_task;
			rendering_tasks[i].end_index = (i+1)*(num_per_task);

			printf("thread %ld, start index: %ld, end index %ld\n", i, rendering_tasks[i].start_index, rendering_tasks[i].end_index);
		}

		// to ensure that boids are rendered
		rendering_tasks[RENDER_THREADS-1].end_index = num_boids;
		printf("thread %ld, has true end index: %ld\n", (RENDER_THREADS-1), num_boids);
	}

	// setup the calculation tasks

	size_t num_per_sim_task;
	if (num_boids < SIMULATION_THREADS)
	{
		// for such a small amount of boids, we can just use one thread
		num_per_sim_task = num_boids;
		sim_tasks[0].start_index = 0;
		sim_tasks[0].end_index = num_boids;
		printf("\n\nSimulation: boids less than threads, start index: %ld, end index: %ld", 0, num_boids);
		// do not use the other tasks
		for (size_t i = 1; i < SIMULATION_THREADS; i++)
		{
			sim_tasks[i].start_index = 0;
			sim_tasks[i].end_index = 0;
		}
	}
	else
	{
		printf("\n\nSimulation Thread distribution:\n");
		size_t num_per_sim_task = num_boids / SIMULATION_THREADS;
		for (size_t i = 0; i < SIMULATION_THREADS; i++)
		{
			sim_tasks[i].start_index = i * num_per_sim_task;
			sim_tasks[i].end_index = (i + 1) * (num_per_sim_task);

			printf("thread %ld, start index: %ld, end index %ld\n", i, sim_tasks[i].start_index, sim_tasks[i].end_index);
		}

		// to ensure that boids are rendered
		sim_tasks[SIMULATION_THREADS - 1].end_index = num_boids;
		printf("thread %ld, has true end index: %ld\n", (SIMULATION_THREADS - 1), num_boids);
	}
	// print parameters
	printf("\n\nr_max: %4.1f,\nbins_x: %4ld, bins_y: %4ld, bins_z: %4ld\norigin_x: %5.1f, origin_y: %5.1f, origin_z: %5.1f\nnum_per_bin: %4ld\nx_jump: %4ld, y_jump: %4ld, z_jump: %4ld\nnum_bins: %8ld\n",
	max_r,
	bins_x,
	bins_y,
	bins_z,
	origin_x,
	origin_y,
	origin_z,
	num_per_bin,
	x_jump,
	y_jump,
	z_jump,
	num_bins
	);

	// reset the boid positions
	ResetBoidPositions();
}

void BoidSimulation::SetupWalls()
{
	// initialize the wall vector
	planes.resize(6);
	// for each wall set a point and a normal
	// the x+ wall
	glm::vec3 p(bound_x, 0.f, 0.f);
	glm::vec3 n(-1.0f, 0.f, 0.f);
	planes[0].point = p;
	planes[0].normal = n;

	// the x- wall
	p = glm::vec3(-bound_x, 0.f, 0.f);
	n = glm::vec3(1.0f, 0.f, 0.f);
	planes[1].point = p;
	planes[1].normal = n;

	// the y+ wall
	p = glm::vec3(0.0f, bound_y, 0.f);
	n = glm::vec3(0.0f, -1.f, 0.f);
	planes[2].point = p;
	planes[2].normal = n;

	// the y- wall
	p = glm::vec3(0.f, -bound_y, 0.f);
	n = glm::vec3(1.0f, 1.f, 0.f);
	planes[3].point = p;
	planes[3].normal = n;

	// the z+ wall
	p = glm::vec3(0.f, 0.f, bound_z);
	n = glm::vec3(0.0f, 0.f, -1.0f);
	planes[4].point = p;
	planes[4].normal = n;

	// the z- wall
	p = glm::vec3(0.0f, 0.f, -bound_z);
	n = glm::vec3(0.0f, 0.f, 1.f);
	planes[5].point = p;
	planes[5].normal = n;


	// create the line wireframe view of the bounding box
	glm::vec3 p1(bound_x, -bound_y, -bound_z);
	glm::vec3 p2(bound_x, -bound_y, bound_z);
	glm::vec3 p3(bound_x, bound_y, bound_z);
	glm::vec3 p4(bound_x, bound_y, -bound_z);

	glm::vec3 p5(-bound_x, -bound_y, -bound_z);
	glm::vec3 p6(-bound_x, -bound_y, bound_z);
	glm::vec3 p7(-bound_x, bound_y, bound_z);
	glm::vec3 p8(-bound_x, bound_y, -bound_z);

	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p1), givr::geometry::Point2(p2)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p2), givr::geometry::Point2(p3)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p3), givr::geometry::Point2(p4)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p4), givr::geometry::Point2(p1)));

	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p5), givr::geometry::Point2(p6)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p6), givr::geometry::Point2(p7)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p7), givr::geometry::Point2(p8)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p8), givr::geometry::Point2(p5)));

	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p4), givr::geometry::Point2(p8)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p3), givr::geometry::Point2(p7)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p1), givr::geometry::Point2(p5)));
	wall_geometry.push_back(givr::geometry::Line(givr::geometry::Point1(p2), givr::geometry::Point2(p6)));

	givr::updateRenderable(wall_geometry, wall_style, wall_render);
}

void BoidSimulation::SetupSpheres()
{
	// resize spheres vector
	spheres.resize(NUM_SPHERES);

	// place 8 at the corners of the volume
	int max = RAND_MAX;

	float r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	glm::vec3 p(bound_x, bound_y, bound_z);
	spheres[0].radius = r;
	spheres[0].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(bound_x, bound_y, -bound_z);
	spheres[1].radius = r;
	spheres[1].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(bound_x, -bound_y, bound_z);
	spheres[2].radius = r;
	spheres[2].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(bound_x, -bound_y, -bound_z);
	spheres[3].radius = r;
	spheres[3].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(-bound_x, bound_y, bound_z);
	spheres[4].radius = r;
	spheres[4].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(-bound_x, bound_y, -bound_z);
	spheres[5].radius = r;
	spheres[5].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(-bound_x, -bound_y, bound_z);
	spheres[6].radius = r;
	spheres[6].point = p;

	r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;
	p = glm::vec3(-bound_x, -bound_y, -bound_z);
	spheres[7].radius = r;
	spheres[7].point = p;


	// the remaining spheres can be at random positions
	for(size_t i = 8; i < NUM_SPHERES; i++)
	{
		r = ((float(std::rand() % max) / float(max))) * (SPHERE_MAXR - SPHERE_MINR) + SPHERE_MINR;

		// generate random position for each sphere
		float x = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_x;
		float y = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_y;
		float z = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_z;

		p = glm::vec3(x,y,z);

		spheres[i].point = p;
		spheres[i].radius = r;
	}
}

void BoidSimulation::PlaceCollidersOnGrid()
{

	// reset the colliders map
	collider_map.clear();
	// iterate over center positions of bins
	for(size_t x_idx = 0; x_idx < bins_x; x_idx++)
	{
		// calculate the x position (x index * bin length + 1/2 bin length + origin_x)
		float x = x_idx*max_r + max_r*0.5f + origin_x;
		for(size_t y_idx = 0; y_idx < bins_y; y_idx++)
		{
			float y = y_idx * max_r + max_r * 0.5f + origin_y;
			for(size_t z_idx = 0; z_idx < bins_z; z_idx++)
			{
				float z = z_idx * max_r + max_r * 0.5f + origin_z;
				
				// define the position of the center of the current grid cube
				glm::vec3 p(x,y,z);

				// the cell index
				size_t cell = x_idx*x_fact + y_idx*y_fact + z_idx;
				// start with the walls
				for(size_t i =0; i < 6; i++)
				{
					// get the displacement from the cube position to a point on the wall
					glm::vec3 d = p - planes[i].point;
					// the distance magnitude from the plane
					float dist = glm::abs(glm::dot(d, planes[i].normal));
					
					// if the distance is within the offset + 0.5 * max_r then the plane is on it
					if(dist < (offset + max_r))
					{
						// add this collider to this gird points array of colliders
						collider_map[cell].push_back(&planes[i]);
					}
				}


				// now the spheres
				for(size_t i = 0; i < spheres.size(); i++)
				{
					// get the distance from the sphere center to the point
					glm::vec3 d = p - spheres[i].point;
					// the squared distance
					float dist_sq = d.x * d.x + d.y * d.y + d.z*d.z;

					// the squared bounding radius
					float bound_sq = (spheres[i].radius + offset + max_r);
					bound_sq = bound_sq*bound_sq;

					if(dist_sq < bound_sq)
					{
						collider_map[cell].push_back(&spheres[i]);
					}
				}
			}
		}
	}

	/*
	// print the map
	for(std::unordered_map<size_t, std::vector<Collider*>>::iterator it = collider_map.begin(); it != collider_map.end(); it++)
	{
		glm::vec3 p;
		if(it->second[0]->type == ColliderType::C_Plane)
		{
			Plane* c = static_cast<Plane*>(it->second[0]);
			p = c->point;
		}
		else
		{
			Sphere* c = static_cast<Sphere*>(it->second[0]);
			p = c->point;
		}

		printf("cell: %ld, contains collider with point: %5.1f, %5.1f, %5.1f\n", it->first, p.x, p.y, p.z);
	}
	*/
	printf("number of cells with colliders: %d\n", collider_map.size());
}

void BoidSimulation::ResetBoidPositions()
{
	// loop through each of the arrays and assign random positions to each boid
	int max = RAND_MAX;

	for(size_t i = 0; i < num_boids; i++)
	{
		// generate random position for each boid
		float x = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_x;
		float y = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_y;
		float z = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_z;

		float vx = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * max_speed*0.5f;
		float vy = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * max_speed*0.5f;
		float vz = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * max_speed*0.5f;

		glm::vec3 p(x,y,z);

		glm::vec3 v = glm::vec3(vx, vy, vz);

		boids_a[i].p = p;
		boids_a[i].v = v;
		boids_b[i].p = p;
		boids_b[i].v = v;
		boids_c[i].p = p;
		boids_c[i].v = v;
	}
}

void BoidSimulation::SortIntoGrid()
{
	// set all grid fill values to zero
	for(size_t i = 0; i < (bins_x*x_fact); i++)
	{
		size_t cell = i * num_per_bin;
		uniform_grid[cell] = 0;
	}

	// for each boid, place it into an appropriate cell on the uniform grid
	for(size_t i = 0; i < num_boids; i++)
	{
		Boid* boid = &(*to_positions)[i];
		glm::vec3 p = boid->p;
		// convert the position to the cell grid position
		size_t index_x = (p.x - origin_x) * max_r_inv;
		size_t index_y = (p.y - origin_y) * max_r_inv;
		size_t index_z = (p.z - origin_z) * max_r_inv;

		size_t cell = index_x * x_fact + index_y * y_fact + index_z;
		boid->cell_index = cell;

		// place this value in the uniform grid
		// calculate the position of the first index in the cell
		cell = cell*num_per_bin;
		// the fill is located at the first index in the cell
		uint32_t fill = ++uniform_grid[cell];
		
		// if the offset is greater that or equal the available positions in the cell
		// then travers in the z direction until we find a cell with vacancy
		if(fill >= num_per_bin)
		{
			while(fill >= num_per_bin)
			{
				uniform_grid[cell]--;
				cell = (cell+num_per_bin)%num_bins;
				fill = ++uniform_grid[cell];
			}
		}

		// place the index of this boid into this cell
		uniform_grid[cell + fill] = i;
	}

	/*
	// print the results
	for(size_t i = 0; i < bins_x*x_fact; i++)
	{
		size_t cell = i * num_per_bin;
		size_t fill = uniform_grid[cell];
		if(fill > 1)
		{
			printf("\ncell: %d contains: ", i); 
			for(size_t j = cell+1; j <= cell+fill; j++)
			{
				printf("%d, ", uniform_grid[j]);
			}
		}
	}
	*/
}

bool BoidSimulation::isValid() const
{
	return true;
}

namespace glm {
	bool isallfinite(glm::vec3 v) { return glm::all(glm::isfinite(v)); }
}

bool Boid::isValid() const { return glm::isallfinite(p) && glm::isallfinite(v); } // velocity only matters if it's not fixed (logically)
bool allValid(const std::vector<Boid> &boids)
{
	return std::all_of(boids.begin(), boids.end(), [](const Boid &boid)
					   { return boid.isValid(); });
}

bool Plane::isValid() const { return true; }
bool allValid(const std::vector<Plane> &planes)
{
	return std::all_of(planes.begin(), planes.end(), [](const Plane &plane)
					   { return plane.isValid(); });
}

bool Sphere::isValid() const { return true; }
bool allValid(const std::vector<Sphere> &spheres)
{
	return std::all_of(spheres.begin(), spheres.end(), [](const Sphere &sphere)
					   { return sphere.isValid(); });
}

namespace simulation {
	namespace primatives {



	}// namespace primatives

	namespace models {

		BoidsModel::BoidsModel()
			: boid_geometry(givr::geometry::Mesh(givr::geometry::Filename("./models/dart.obj")))
			, boid_style(givr::style::Colour(1.f, 1.f, 0.f), givr::style::LightPosition(100.f, 100.f, 100.f))
			, wall_geometry()
			, wall_style(givr::style::Colour(0.f, 1.0f, 0.0f))
			, sphere_geometry(givr::geometry::Radius(1.0f))
			, sphere_style(givr::style::Colour(1.0f, 0.0f, 1.0f), givr::style::LightPosition(100.0f, 100.0f, 100.0f))
		{
			// Reset Dynamic elements
			reset();

			// create a triangle for the wall
			glm::vec3 v1 = glm::vec3(0.0f,0.0f,0.0f);
			glm::vec3 v2 = glm::vec3(1.0f, 0.0f, 0.0f);
			glm::vec3 v3 = glm::vec3(0.0f, 0.0f, 1.0f);

			wall_geometry.push_back(
				givr::geometry::Line(givr::geometry::Point1(v1), givr::geometry::Point2(v2))
			);


			// Render
			boid_render = givr::createInstancedRenderable(boid_geometry, boid_style);
			wall_render = givr::createRenderable(wall_geometry, wall_style);
			sphere_render = givr::createInstancedRenderable(sphere_geometry, sphere_style);
		}

		void BoidsModel::reset() {
			// static so they are persistent
			static std::random_device random_device;
			static std::mt19937 generator(random_device());
			static std::uniform_real_distribution<double> position_distribution(-10., 10.);
			static std::uniform_real_distribution<double> theta_distribution(-180., 180.);
			static std::uniform_real_distribution<double> phi_distribution(-90., 90.);
			static std::uniform_real_distribution<double> speed_distribution(5., 25.);

			boids.resize(n_boids);
			for (Boid& boid : boids) {
				//Random Position in 20 x 20 x 20 cube
				boid.p.x = position_distribution(generator);
				boid.p.y = position_distribution(generator);
				boid.p.z = position_distribution(generator);

				//Random heading (two angles) and Random speed
				double theta = glm::radians(theta_distribution(generator));
				double phi = glm::radians(phi_distribution(generator));
				double speed = speed_distribution(generator);

				// https://stackoverflow.com/questions/30011741/3d-vector-defined-by-2-angles
				boid.v.x = speed * std::cos(theta) * std::cos(phi);
				boid.v.y = speed * std::sin(phi);
				boid.v.z = speed * std::sin(theta) * std::cos(phi);

			}
		}

		void BoidsModel::step(float dt) {
			reset(); // Comment this out
			//TODO: apply forces and compute integration step
		}

		void BoidsModel::render(const ModelViewContext& view) {
			//Add Mass render
			for (const Boid& boid : boids)
				givr::addInstance(boid_render, glm::translate(glm::mat4(1.f), boid.p)); //NEED TO FRAME!!!
			
			givr::addInstance(sphere_render, glm::mat4(1.0f));

			//Render
			givr::style::draw(wall_render, view);
			givr::style::draw(boid_render, view);
			givr::style::draw(sphere_render, view);
		}

		bool BoidsModel::isValid() const {
			return allValid(boids); // && primatives::allValid(planes) && primatives::allValid(spheres);
		}

	} // namespace models
} // namespace simulation