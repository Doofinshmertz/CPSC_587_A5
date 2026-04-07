/**
 * CPSC 587 W26 Assignment 5
 * @name Holden Holzer
 * @email holden.holzer@ucalgary.ca
 *
 * Modified from provided Assignment 5 - Boilerplate
 * @authors Copyright 2019 Lakin Wecker, Jeremy Hart, Andrew Owens and Others (see AUTHORS)
 */

#include "models.hpp"
#include <random>
#include <stdio.h>
#include <chrono>
#include <cmath>
// constructor
BoidSimulation::BoidSimulation()
	: boid_geometry(givr::geometry::Mesh(givr::geometry::Filename("./models/dart.obj"))),
	boid_style(givr::style::Colour(1.f, 1.f, 0.f), givr::style::LightPosition(1000.f, 1000.f, 1000.f)),
	wall_geometry(), wall_style(givr::style::Colour(0.f, 1.0f, 0.0f)),
	sphere_geometry(givr::geometry::Radius(1.0f)),
	sphere_style(givr::style::Colour(1.0f, 0.0f, 1.0f), givr::style::LightPosition(1000.0f, 1000.0f, 1000.0f))
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
	float _bounds[3],
	int _num_spheres, float _sphere_max_r, float _sphere_min_r,
	size_t _num_boids)
{



	// need to re-calculate grid spacing
	// re-initialze the boids

	// if a simulation was already running, restart it, otherwise do not
	if((stop_simulation == false) || main_loop_thread.joinable())
	{
		printf("re setting setings\n");
		// set stop simulation to true
		stop_simulation = true;

		main_loop_thread.join();

		array_mutex.lock();
		// reset this when the simulation is re-started
		last_render_time = std::chrono::steady_clock::now();

		// set all values
		angle_sep = std::cos(glm::radians(_angle_sep));
		angle_align = std::cos(glm::radians(_angle_align));
		angle_coh = std::cos(glm::radians(_angle_coh));
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
		num_boids = _num_boids; // this was one hell of a race condition to find

		bound_x = _bounds[0];
		bound_y = _bounds[1];
		bound_z = _bounds[2];

		num_spheres = _num_spheres;
		sphere_max_r = _sphere_max_r;
		sphere_min_r = _sphere_min_r;

		printf("main loop should have stopped\n");
		SetupSimulation();

		array_mutex.unlock();
		stop_simulation = false;
		// start the simulation loop
		main_loop_thread = std::thread(&BoidSimulation::SimulationLoop, this);
	}
	else
	{
		// reset this when the simulation is re-started
		last_render_time = std::chrono::steady_clock::now();
		// set all values
		angle_sep = std::cos(glm::radians(_angle_sep));
		angle_align = std::cos(glm::radians(_angle_align));
		angle_coh = std::cos(glm::radians(_angle_coh));
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
		num_boids = _num_boids; // this was one hell of a race condition to find

		bound_x = _bounds[0];
		bound_y = _bounds[1];
		bound_z = _bounds[2];

		num_spheres = _num_spheres;
		sphere_max_r = _sphere_max_r;
		sphere_min_r = _sphere_min_r;

		printf("starting simulation\n");
		// reset the boid positions
		SetupSimulation();
		// start a new simulation
		stop_simulation = false;
		// start the simulation loop
		main_loop_thread = std::thread(&BoidSimulation::SimulationLoop, this);
	}

	printf("angle_sep: %f, angle align: %f, angle coh: %f\n, k_sep: %f\n", angle_sep, angle_align, angle_coh, k_sep);
}

/**
 * stop the current thread, then reset the positions of the boids
 */
void BoidSimulation::reset()
{
	// set stop simulation to true
	stop_simulation = true;

	// reset this when the simulation is re-started
	last_render_time = std::chrono::steady_clock::now();

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
	return;
}

void BoidSimulation::SimulationLoop()
{
	// setup time measurement variables
	std::chrono::steady_clock::time_point start_t = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end_t = start_t;
	delta_time = 0.01f; // starting delta time, we don't what this to be equal to zero

	// while stop simulation is not true, 
	while(!stop_simulation)
	{
		// if the simulation is paused, sleep for some time, then try again
		if(is_paused)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			continue;
		}

		
		// acceleration step
		// dispatch threads
		float del = delta_time;
		for (size_t i = 0; i < SIMULATION_THREADS; i++)
		{
			threads_sim[i] = std::thread(&BoidSimulation::SimulateBoids, this, i, del);
		}
		

		// join threads
		for (size_t i = 0; i < SIMULATION_THREADS; i++)
		{
			threads_sim[i].join();
		}
		

		//ResetBoidPositions();
		
		// update the grid
		UpdateHashGrid();
		array_mutex.lock();
		// swap buffers
		std::vector<Boid>* temp = start_positions.load();
		start_positions = end_positions.load();
		end_positions = to_positions.load();
		to_positions = temp;

		array_mutex.unlock();
		// reset the elapsed time
		elapsed_time = 0.0f;
		

		// measure the time step
		end_t = std::chrono::steady_clock::now();
		delta_time = (std::chrono::duration<float, std::ratio<1>>(end_t - start_t)).count();
		start_t = end_t;
	}
}

void BoidSimulation::UpdateHashGrid()
{
	// set the fill values to zero
	for (size_t i = 0; i < num_cells; i++)
	{
		hash_grid[i].clear();
		hash_grid[i].reserve(num_per_cell);
	}
	
	// iterate over the boids and place them in appropriate cells
	for (size_t i = 0; i < num_boids; i++)
	{
		// the get cell value of the current boid
		size_t cell_index = (*to_positions)[i].cell_index % num_cells;
		// insert into the hash grid
		for (int x = -1; x <= 1; x++)
		{
			for (int y = -1; y <= 1; y++)
			{
				for (int z = -1; z <= 1; z++)
				{
					int32_t key = (cell_index + x * x_fact + y * y_fact + z) % num_cells;
					if(hash_grid[key].size() < MAX_NEIGHBORS)
					{
						hash_grid[key].push_back(i);
					}
				}
			}
		}
		
	}
}

void BoidSimulation::SimulateBoids(size_t thread_id, float dt)
{
	size_t start = sim_tasks[thread_id].start_index;
	size_t end = sim_tasks[thread_id].end_index;

	if(end == start){return;}

	std::vector<Boid> *boids_to = to_positions.load();
	std::vector<Boid> *boids_end = end_positions.load();

	// detection distance for objects
	float object_detect_r = max_r + offset;

	// detection distance for boids
	float boid_detect_r = max_r * max_r;

	for(size_t i = start; i < end; i++)
	{

		//printf("tr: %lu, checkpoint 1\n", thread_id);

		// get the current boid
		Boid *boid = &(*end_positions)[i];
		int64_t cell = boid->cell_index;

		// the acceleration accumulator
		glm::vec3 acc_obs(0.0f, 0.0f, 0.0f);
		
		// check the cell along with the surrounding cells
		// iterate over the x coordinate
		//printf("tr: %lu, checkpoint 2\n", thread_id);
		for(int32_t x_idx = -x_fact; x_idx <= x_fact; x_idx += x_fact)
		{
			// iterate over the y coordinate
			for(int32_t y_idx = -y_fact; y_idx <= y_fact; y_idx += y_fact)
			{

				// iterate over the z coordinate
				for(int32_t z_idx = -1; z_idx <= 1; z_idx++)
				{
					// the cell key
					size_t key = (cell + x_idx + y_idx +z_idx)%num_cells;
					//printf("tr: %lu, checkpoint 3\n", thread_id);
					//std::cout << std::flush;
					// check for colliders first
					if(collider_map.contains(key))
					{
						//printf("tr: %lu, checkpoint 4\n", thread_id);
						//std::cout << std::flush;
						std::vector<Collider*>* colliders = &collider_map[key];

						for(uint32_t j = 0; j < colliders->size(); j++)
						{
							//printf("tr: %lu, checkpoint 5\n", thread_id);
							//std::cout << std::flush;
							Collider* ptr = (*colliders)[j];
							// iterate over potential colliders at this position
							if (ptr->type == ColliderType::C_Plane)
							{
								Plane *plane = static_cast<Plane *>(ptr);
								//printf("tr: %lu, checkpoint 6\n", thread_id);
								//std::cout << std::flush;
								// check the distance
								float dist_sqr = glm::dot((boid->p - plane->point), plane->normal);
								//printf("tr: %lu, checkpoint 6.1\n", thread_id);
								//std::cout << std::flush;

								dist_sqr = dist_sqr* dist_sqr;

								//printf("tr: %lu, checkpoint 6.2\n", thread_id);
								//std::cout << std::flush;
								if(dist_sqr < (object_detect_r*object_detect_r))
								{

									// is the boid within the offset distance of the wall, if so use repulsion
									if (dist_sqr < (offset * offset))
									{
										// if within offset distance, apply max acceleration to prevent collision
										acc_obs += plane->normal * max_acc;
									}
									else
									{
										// otherwise use turning

										// first calculate the acceleration normal
										// the velocity maginutude
										float v_m = glm::length(boid->v);
										// the velocity normal
										glm::vec3 v_n = boid->v / v_m;

										// if the boid is facing away from the plane then don't bother steering
										float direction = -glm::dot(v_n, plane->normal);
										if(direction > 0)
										{
											// the normal of the plane
											glm::vec3 n = plane->normal;

											// the acceleration normal
											glm::vec3 a = n - glm::dot(v_n, n) * v_n;
											float mag = a.x * a.x + a.y * a.y + a.z * a.z;
											// if velocity is pointing directly at the normal, choose a different vector to get it out of this scenario
											if (mag < 0.0001f)
											{
												a = glm::vec3(n.y, n.z, n.x);
												mag = a.x * a.x + a.y * a.y + a.z * a.z;
											}

											mag = glm::sqrt(mag);
											a = a / mag;

											// now calculate the radius
											// the displacement vector
											glm::vec3 disp = boid->p - plane->point;

											float r = (glm::dot(disp, n) - offset) / (1.0f - glm::dot(a, n));

											float a_m = v_m * v_m / r;
											if (std::isnan(a_m) || std::isinf(a_m) || a_m > max_acc)
											{
												a_m = max_acc;
											}
											acc_obs += a * a_m;
										}
									}
								}
								//printf("tr: %lu, checkpoint 7\n", thread_id);
							}
							else
							{
								Sphere *sphere = static_cast<Sphere *>(ptr);
								//printf("tr: %lu, checkpoint 8\n", thread_id);

								// check the distance
								glm::vec3 pc = boid->p - sphere->point;

								float dist_sqr = pc.x*pc.x+pc.y*pc.y + pc.z*pc.z;
								// the detection radius of the boid + the offset + the radius of the sphere
								float detect_dist = object_detect_r + sphere->radius;
								
								if(dist_sqr < detect_dist*detect_dist)
								{
									// is the boid within the offset distance of the sphere, if so use repulsion
									float repulsion_dist = sphere->radius + offset;
									float dist = glm::sqrt(dist_sqr);
									if (dist < repulsion_dist)
									{
										// use repulsion
										glm::vec3 normal = boid->p - sphere->point;
										normal = normal / dist;
										// if within offset distance, apply max acceleration to prevent collision
										acc_obs += normal * max_acc;
									}
									else
									{
										// use streering
										// first calculate the acceleration normal
										// get the velocity
										glm::vec3 v_n = boid->v;
										float v_m = glm::length(v_n);
										v_n = v_n / v_m;

										// calculate if we are going to hit the sphere

										// positive if we are facing the sphere
										float v_dot_dist = -glm::dot(v_n, pc);
										if(v_dot_dist > 0.0f)
										{
											// does our path intersect the sphere
											float pass_sqr = dist_sqr - (v_dot_dist * v_dot_dist);
											if(pass_sqr < (detect_dist*detect_dist))
											{
												glm::vec3 pc_n = glm::normalize(pc);

												// the acceleration normal
												glm::vec3 a_n = v_n - glm::dot(v_n, pc_n) * pc_n;
												float mag = a_n.x * a_n.x + a_n.y * a_n.y + a_n.z * a_n.z;
												// if velocity is pointing directly at the normal, choose a different vector to get it out of this scenario
												if (mag < 0.0001f)
												{
													a_n = glm::vec3(pc_n.y, pc_n.z, pc_n.x);
													mag = a_n.x * a_n.x + a_n.y * a_n.y + a_n.z * a_n.z;
												}

												mag = glm::sqrt(mag);
												a_n = a_n / mag;

												// now calculate the radius
												float surface = sphere->radius + offset;
												float r = ((pc.x * pc.x + pc.y * pc.y + pc.z * pc.z) - surface * surface) / (2.0f * (surface - glm::dot(a_n, pc)));

												float a_m = v_m * v_m / r;
												if (std::isnan(a_m) || std::isinf(a_m) || a_m > max_acc)
												{
													a_m = max_acc;
												}
												acc_obs += a_m * a_n;
											}
										}
									}
								}
								//printf("tr: %lu, checkpoint 9\n", thread_id);
								//std::cout << std::flush;
							}
						}
					}

					//printf("tr: %lu, checkpoint 9.1\n", thread_id);
					//std::cout << std::flush;
				}
			}
		}
		// printf("tr: %lu, checkpoint 9.2\n", thread_id);
		// std::cout << std::flush;

		//  count the number of neighbors
		int32_t num_neighbors = 0;
		
		
		
		//printf("tr: %lu, checkpoint 10\n", thread_id);
		//std::cout << std::flush;

		
		std::vector<uint32_t>* neighbors = &hash_grid[(boid->cell_index) % num_cells];

		glm::vec3 acc_bo(0.0f, 0.0f, 0.0f);
		size_t n = neighbors->size();
		for(size_t j = 0; j <n; j++)
		{
			// if the number of neighbors exceeds a certian number the stop
			if (num_neighbors >= MAX_NEIGHBORS)
			{
				break;
			}

			// printf("acc_urg: %f, %f, %f\n", acc_urg.x, acc_urg.y, acc_urg.z);
			Boid *other = &((*boids_end)[(*neighbors)[j]]);
			if(other == boid){continue;}
			glm::vec3 disp = other->p - boid->p;
			float dist_sqr = disp.x*disp.x + disp.y*disp.y + disp.z*disp.z;
			if(dist_sqr < (max_r * max_r))
			{
				acc_bo += HandleBoid(boid, other, dist_sqr);
				num_neighbors++;
			}
		}
		
		glm::vec3 acc = acc_obs;

		if(num_neighbors > 0)
		{
			acc += (1.0f / float(num_neighbors)) * acc_bo;
		}
		
		//printf("tr: %lu, checkpoint 12.1\n", thread_id);
		//std::cout << std::flush;
		// bound the acceleration to a maximum value
		float acc_m = acc.x * acc.x + acc.y * acc.y + acc.z * acc.z;
		if (acc_m > (max_acc * max_acc))
		{
			acc = acc * max_acc / glm::sqrt(acc_m);
		}
		//printf("tr: %lu, checkpoint 13\n", thread_id);
		//std::cout << std::flush;

		// calculate the new velocity
		glm::vec3 v = boid->v + acc*dt;

		// bound the new velocity
		float v_m = v.x * v.x + v.y * v.y + v.z * v.z;
		if(v_m > (max_speed * max_speed))
		{
			v = v * max_speed / glm::sqrt(v_m);
		}
		//printf("tr: %lu, checkpoint 14\n", thread_id);
		//std::cout << std::flush;

		// calculate the new position
		glm::vec3 p = boid->p + v*dt;

		// bound the new position
		if(p.x > bound_x)
		{
			p.x = bound_x - offset;
			//printf("x_positive bound hit\n");
		}
		if(p.x < -bound_x)
		{
			p.x = offset - bound_x;
			//printf("x negative bound hit\n");
		}
		//printf("tr: %lu, checkpoint 15\n", thread_id);

		if (p.y > bound_y)
		{
			p.y = bound_y - offset;
			//printf("y positive bound hit\n");
		}
		if (p.y < -bound_y)
		{
			p.y = offset - bound_y;
			//printf("y negative bound hit\n");
		}
		//printf("tr: %lu, checkpoint 16\n", thread_id);

		if (p.z > bound_z)
		{
			p.z = bound_z - offset;
			//printf("z positive bound hit\n");
		}
		if (p.z < -bound_z)
		{
			p.z = offset - bound_z;
			//printf("z negative bound hit\n");
		}
		//printf("tr: %lu, checkpoint 16.1\n", thread_id);
		//std::cout << std::flush;
		// get the new boid
		Boid *boid_to = &(*boids_to)[i];
		//printf("tr: %lu, checkpoint 17\n", thread_id);
		//std::cout << std::flush;

		boid_to->v = v;
		boid_to->p = p;
		//printf("tr: %lu, checkpoint 18\n", thread_id);
		//std::cout << std::flush;

		// calculate the new cell
		// convert the position to the cell grid position
		size_t index_x = (p.x - origin_x) * max_r_inv;
		size_t index_y = (p.y - origin_y) * max_r_inv;
		size_t index_z = (p.z - origin_z) * max_r_inv;
		//printf("tr: %lu, checkpoint 18\n", thread_id);

		boid_to->cell_index = (index_x * x_fact + index_y * y_fact + index_z)%num_cells;
		//printf("tr: %lu, checkpoint 19\n", thread_id);
		//std::cout << std::flush;
	}
}


glm::vec3 BoidSimulation::HandlePlane(const Boid *boid, const Plane *plane, float dist_sqr)
{

	// is the boid within the offset distance of the wall, if so use repulsion
	if(dist_sqr < (offset*offset))
	{
		// if within offset distance, apply max acceleration to prevent collision	
		return plane->normal * max_acc;
	}

	
	// otherwise use turning

	// first calculate the acceleration normal
	// the velocity maginutude
	float v_m = glm::length(boid->v);
	// the velocity normal
	glm::vec3 v_n = boid->v / v_m;

	// the normal of the plane
	glm::vec3 n = plane->normal;

	// the acceleration normal
	glm::vec3 a = n - glm::dot(v_n, n) * v_n;
	float mag = a.x * a.x + a.y*a.y + a.z*a.z;
	// if velocity is pointing directly at the normal, choose a different vector to get it out of this scenario
	if(mag < 0.0001f)
	{
		a = glm::vec3(n.y, n.z, n.x);
		mag = a.x * a.x + a.y * a.y + a.z * a.z;
	}

	mag = glm::sqrt(mag);
	a = a/mag;

	// now calculate the radius
	// the displacement vector
	glm::vec3 disp = boid->p - plane->point;
	
	float r = (glm::dot(disp, n) - offset) / (1.0f - glm::dot(a, n));

	float a_m = v_m*v_m / r;
	if(std::isnan(a_m) || std::isinf(a_m) || a_m > max_acc)
	{
		a_m = max_acc;
	}
	return a * a_m;
}

glm::vec3 BoidSimulation::HandleSphere(const Boid *boid, const Sphere *sphere, float dist_sqr)
{
	// is the boid within the offset distance of the sphere, if so use repulsion
	float repulsion_dist = sphere->radius + offset;
	float dist = glm::sqrt(dist_sqr);
	if (dist < repulsion_dist)
	{
		// use repulsion
		glm::vec3 normal = boid->p - sphere->point;
		normal = normal / dist;
		// if within offset distance, apply max acceleration to prevent collision
		return normal * max_acc;
	}
	else
	{
		// use streering
		// first calculate the acceleration normal
		// get the velocity
		glm::vec3 v_n = boid->v;
		float v_m = glm::length(v_n);
		v_n = v_n / v_m;

		// the displacement vector
		glm::vec3 pc = boid->p - sphere->point;
		glm::vec3 pc_n = glm::normalize(pc);

		// the acceleration normal
		glm::vec3 a_n = v_n - glm::dot(v_n, pc_n) * pc_n;
		float mag = a_n.x * a_n.x + a_n.y * a_n.y + a_n.z * a_n.z;
		// if velocity is pointing directly at the normal, choose a different vector to get it out of this scenario
		if (mag < 0.0001f)
		{
			a_n = glm::vec3(pc_n.y, pc_n.z, pc_n.x);
			mag = a_n.x * a_n.x + a_n.y * a_n.y + a_n.z * a_n.z;
		}

		mag = glm::sqrt(mag);
		a_n = a_n / mag;

		// now calculate the radius
		float surface = sphere->radius + offset;
		float r = ((pc.x * pc.x + pc.y * pc.y + pc.z * pc.z) - surface * surface) / (2.0f * (surface - glm::dot(a_n, pc)));

		float a_m = v_m * v_m / r;
		if (std::isnan(a_m) || std::isinf(a_m) || a_m > max_acc)
		{
			a_m = max_acc;
		}
		return a_m * a_n;
	}
}

glm::vec3 BoidSimulation::HandleBoid(const Boid *boid, const Boid *other, float dist_sqr)
{
	// acceleration accumulator
	glm::vec3 acc(0.0f, 0.0f, 0.0f);

	// displacement vector
	glm::vec3 disp = other->p - boid->p;

	// cosine view angle
	float v_m = glm::length(boid->v);
	float dist = glm::sqrt(dist_sqr);
	float view_angle = glm::dot(boid->v, disp) / (v_m * dist);

	// out of range, ignor this
	if(dist > r_coh)
	{
		return glm::vec3(0.0f, 0.0f, 0.0f);
	}

	// case for cohesion
	if((dist > r_align) && (view_angle > angle_coh))
	{
		acc = k_coh * disp;
		return acc;
	}
	// apply alignment
	else if ((dist > r_sep) && (view_angle > angle_align))
	{
		acc = k_align * (other->v - boid->v);
		return acc;
	}
	// case for separation (do not apply cohesion while separating
	else if(view_angle > angle_sep)
	{
		float acc_m = k_sep / (dist_sqr);
		if(std::isnan(acc_m) || std::isinf(acc_m))
		{
			return glm::vec3(max_acc, 0.0f, 0.0f);
		}

		acc = (-1.0f) * disp * acc_m;
		return acc;
	}

	return glm::vec3(0.0f, 0.0f, 0.0f);
}

float BoidSimulation::GetDeltatime()
{
	return delta_time;
}

void BoidSimulation::render(const ModelViewContext &view)
{
	// we are going to be reading from the arrays so do not let them get swaped
	array_mutex.lock();

	// render spheres
	for(size_t i = 0; i < num_spheres; i++)
	{
		givr::addInstance(sphere_render, glm::scale( glm::translate(glm::mat4(1.0f), spheres[i].point), glm::vec3(spheres[i].radius, spheres[i].radius, spheres[i].radius)));
	}


	// the interpolation factor
	float interp = (elapsed_time.load() / delta_time.load());
	if(interp > 1.2f)
	{
		interp = 1.2f;
	}

	//printf("delta_time: %5.3f, elapsed_time: %5.3f, interp %6.4f\n",delta_time.load(), elapsed_time.load(), interp);
	// dispatch render threads
	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		threads_rnd[i] = std::thread(&BoidSimulation::AddBoidRenderable, this, i, interp);
	}

	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		threads_rnd[i].join();
	}
	array_mutex.unlock();
	// draw
	givr::style::draw(wall_render, view);
	givr::style::draw(sphere_render, view);
	
	// draw all boids
	for(size_t i = 0; i < RENDER_THREADS; i++)
	{
		givr::style::draw(boid_render[i], view);
	}

	std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
	render_delta = (std::chrono::duration<float, std::ratio<1>>(end_t - last_render_time)).count();
	last_render_time = end_t;
	elapsed_time.store(render_delta + elapsed_time.load());
}

void BoidSimulation::AddBoidRenderable(size_t thread_id, float interp)
{
	size_t start = rendering_tasks[thread_id].start_index;
	size_t end = rendering_tasks[thread_id].end_index;

	std::vector<Boid>* boids_start = start_positions;
	std::vector<Boid>* boids_end = end_positions;

	if(start == end){ return;}
	// render the boids in between the given indices
	for(size_t i = start; i < end; i++)
	{
		// interpolate between these two
		glm::vec3 p_start = (*boids_start)[i].p;
		glm::vec3 v_start = (*boids_start)[i].v;

		glm::vec3 p_end = (*boids_end)[i].p;
		glm::vec3 v_end = (*boids_end)[i].v;

		// calculate the acceleration
		glm::vec3 a_avg = (v_end - v_start) * ((1.0f)/ delta_time);

		// low pass filter with previous values
		a_avg = 0.1f*a_avg + 0.9f*accelerations[i];
		accelerations[i] = a_avg;

		// calculate the new position and velocity
		p_start = (p_end - p_start)*interp + p_start;
		v_start = (v_end - v_start)*interp + v_start;

		p_start = 0.1f*p_start + 0.9f*positions[i];
		positions[i] = p_start;

		v_start = 0.1f*v_start + 0.9f*velocities[i];
		velocities[i] = v_start;
		glm::vec3 gravity(0.0f, -9.81f, 0.0f);
		// the tangents
		glm::vec3 T = glm::normalize(v_start);

		// gravity tangent
		glm::vec3 g_tan = gravity - glm::dot(gravity, T) * T;

		// total acceleration vector
		glm::vec3 N = glm::normalize((a_avg - g_tan));

		// bi-normal
		glm::vec3 B = glm::cross(N, T);

		glm::mat4 M = glm::mat4{1.0f};

		M[0][0] = B.x;
		M[0][1] = B.y;
		M[0][2] = B.z;

		M[1][0] = N.x;
		M[1][1] = N.y;
		M[1][2] = N.z;

		M[2][0] = T.x;
		M[2][1] = T.y;
		M[2][2] = T.z;

		M[3][0] = p_start.x;
		M[3][1] = p_start.y;
		M[3][2] = p_start.z;

	
		M = glm::scale(M, glm::vec3{3.0f});

		givr::addInstance(boid_render[thread_id], M);
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
	accelerations.resize(num_boids);
	positions.resize(num_boids);
	velocities.resize(num_boids);
	// set the arrays for rendering and simulation
	start_positions.store(&boids_a);
	end_positions.store(&boids_b);
	to_positions.store(&boids_c);

	// starting configuration for the renderer
	new_buffers = true;
	elapsed_time = 0.0f;

	// need to setup the uniform grid 
	// calculate the number of bins in each direction

	// get the largest detection radius
	max_r = std::max(std::max(r_sep, r_align), r_coh);
	max_r_inv = 1.0f / max_r;
	// calculate the number of bins in the x,y,z direction (+2 to ensure overlap)
	bins_x = 2 * (size_t(bound_x / max_r) + 2);
	bins_y = 2 * (size_t(bound_y / max_r) + 2);
	bins_z = 2 * (size_t(bound_z / max_r) + 2);

	// calculate the location of the all negative corner of the uniform grid (this will be used as the origin)
	origin_x = -float((bins_x >> 1)) * max_r;
	origin_y = -float((bins_y >> 1)) * max_r;
	origin_z = -float((bins_z >> 1)) * max_r;

	// approximate the number per bin as the average number of boids per cell in the simulation multiplied by 2 plus 10 (x*2 ensures ratio of at least 2, +10 accounts for sparse grid scenario where num_boids/bins is near zero) 
	num_per_cell = ((num_boids / (bins_x * bins_y * bins_z)) + 1)*10 + 1 + 10;

	// the jumps (values needed to jump the grid by one bin)
	x_jump = bins_y * bins_z * num_per_cell;
	y_jump = bins_z*num_per_cell;
	z_jump = num_per_cell;

	// used for finding cell indices
	x_fact = bins_y * bins_z;
	y_fact = bins_z;

	num_cells = x_fact*bins_x;

	ugrid_len = num_cells*num_per_cell;
	hash_grid.clear();
	hash_grid.resize(num_cells);

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
	num_per_cell,
	x_jump,
	y_jump,
	z_jump,
	ugrid_len
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
	planes[0].type = ColliderType::C_Plane;

	// the x- wall
	p = glm::vec3(-bound_x, 0.f, 0.f);
	n = glm::vec3(1.0f, 0.f, 0.f);
	planes[1].point = p;
	planes[1].normal = n;
	planes[1].type = ColliderType::C_Plane;

	// the y+ wall
	p = glm::vec3(0.0f, bound_y, 0.f);
	n = glm::vec3(0.0f, -1.f, 0.f);
	planes[2].point = p;
	planes[2].normal = n;
	planes[2].type = ColliderType::C_Plane;

	// the y- wall
	p = glm::vec3(0.f, -bound_y, 0.f);
	n = glm::vec3(0.0f, 1.f, 0.f);
	planes[3].point = p;
	planes[3].normal = n;
	planes[3].type = ColliderType::C_Plane;

	// the z+ wall
	p = glm::vec3(0.f, 0.f, bound_z);
	n = glm::vec3(0.0f, 0.f, -1.0f);
	planes[4].point = p;
	planes[4].normal = n;
	planes[4].type = ColliderType::C_Plane;

	// the z- wall
	p = glm::vec3(0.0f, 0.f, -bound_z);
	n = glm::vec3(0.0f, 0.f, 1.f);
	planes[5].point = p;
	planes[5].normal = n;
	planes[5].type = ColliderType::C_Plane;

	// create the line wireframe view of the bounding box
	glm::vec3 p1(bound_x, -bound_y, -bound_z);
	glm::vec3 p2(bound_x, -bound_y, bound_z);
	glm::vec3 p3(bound_x, bound_y, bound_z);
	glm::vec3 p4(bound_x, bound_y, -bound_z);

	glm::vec3 p5(-bound_x, -bound_y, -bound_z);
	glm::vec3 p6(-bound_x, -bound_y, bound_z);
	glm::vec3 p7(-bound_x, bound_y, bound_z);
	glm::vec3 p8(-bound_x, bound_y, -bound_z);

	wall_geometry = givr::geometry::MultiLine();

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
	if(num_spheres < 14)
	{
		num_spheres = 14;
	}
	// resize spheres vector
	spheres.resize(num_spheres);

	// place 8 at the corners of the volume
	int max = RAND_MAX;

	float r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	glm::vec3 p(bound_x, bound_y, bound_z);
	spheres[0].radius = r;
	spheres[0].point = p;
	spheres[0].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(bound_x, bound_y, -bound_z);
	spheres[1].radius = r;
	spheres[1].point = p;
	spheres[1].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(bound_x, -bound_y, bound_z);
	spheres[2].radius = r;
	spheres[2].point = p;
	spheres[2].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(bound_x, -bound_y, -bound_z);
	spheres[3].radius = r;
	spheres[3].point = p;
	spheres[3].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(-bound_x, bound_y, bound_z);
	spheres[4].radius = r;
	spheres[4].point = p;
	spheres[4].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(-bound_x, bound_y, -bound_z);
	spheres[5].radius = r;
	spheres[5].point = p;
	spheres[5].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(-bound_x, -bound_y, bound_z);
	spheres[6].radius = r;
	spheres[6].point = p;
	spheres[6].type = ColliderType::C_Sphere;

	r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;
	p = glm::vec3(-bound_x, -bound_y, -bound_z);
	spheres[7].radius = r;
	spheres[7].point = p;
	spheres[7].type = ColliderType::C_Sphere;

	// the remaining spheres can be at random positions
	for(size_t i = 8; i < num_spheres; i++)
	{
		r = ((float(std::rand() % max) / float(max))) * (sphere_max_r - sphere_min_r) + sphere_min_r;

		// generate random position for each sphere
		float x = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_x;
		float y = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_y;
		float z = ((float(std::rand() % max) / float(max)) * 2.0f - 1.0f) * bound_z;

		p = glm::vec3(x,y,z);

		spheres[i].point = p;
		spheres[i].radius = r;
		spheres[i].type = ColliderType::C_Sphere;
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
	for(size_t i = 0; i < (num_cells); i++)
	{
		hash_grid[i].clear();
		hash_grid[i].reserve(num_per_cell);
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

		// insert into the hash grid
		for (int x = -1; x <= 1; x++)
		{
			for (int y = -1; y <= 1; y++)
			{
				for (int z = -1; z <= 1; z++)
				{
					int32_t key = cell + x * x_fact + y * y_fact + z;
					hash_grid[key].push_back(i);
				}
			}
		}
	}


	/*
	// print the hash grid
	for(size_t i = 0; i < num_cells; i++)
	{
		size_t size = hash_grid[i].size();
		size_t index_x = i / x_fact;
		size_t index_y = (i % x_fact) / y_fact;
		size_t index_z = (i % x_fact) % y_fact;

		float x = float(index_x) * max_r + origin_x;
		float y = float(index_y) * max_r + origin_y;
		float z = float(index_z) * max_r + origin_z;

		printf("\nCell: %lu has size: %lu, %4.1f, %4.1f, %4.1f\n", i, size, x, y, z);
		// the neighbors should be:
		for(int x_i = -1; x_i <= 1; x_i++)
		{
			
			for(int y_i = -1; y_i <= 1; y_i++)
			{
				printf("\nCells: ");
				for(int z_i = -1; z_i <= 1; z_i++)
				{
					printf("%lu, ", (i + x_i*x_fact + y_i*y_fact +z_i));
				}
			}
			printf("\n");
		}
		
		for(size_t j = 0; j < size; j++)
		{
			Boid *boid = &(*to_positions)[hash_grid[i][j]];

			printf("\t point: %4.1f, %4.1f, %4.1f, cell: %lu\n", boid->p.x, boid->p.y, boid->p.z, boid->cell_index);
		}
	}
	*/
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
