#pragma once
/**
 * CPSC 587 W26 Assignment 5
 * @name Holden Holzer
 * @email holden.holzer@ucalgary.ca
 *
 * Modified from provided Assignment 5 - Boilerplate
 * @authors Copyright 2019 Lakin Wecker, Jeremy Hart, Andrew Owens and Others (see AUTHORS)
 */

#include <givio.h>
#include <givr.h>
#include <imgui/imgui.h>

namespace imgui_panel {
	extern bool showPanel;
	extern ImVec4 clear_color;
	extern bool reset_view;

	// Simulation settings
	extern int number_of_iterations_per_frame;
	extern bool play_simulation;
	extern bool reset_simulation;
	extern bool step_simulation;
	extern float dt_simulation;
	extern int n_boids;
	extern bool enable_validity_check;

	// Simulation State
	extern bool valid_simulation;
	extern float t_simulation;
	extern int n_frames;
	extern int n_frames_less_than_20;

	// lambda function
	extern std::function<void(void)> draw;

	// custom values
	extern float angle_align;
	extern float angle_sep;
	extern float angle_coh;

	extern float r_align;
	extern float r_sep;
	extern float r_coh;

	extern float k_align;
	extern float k_sep;
	extern float k_coh;

	extern float offset;
	extern float k_repulsion;

	extern float max_speed;
	extern float max_acceleration;

	extern bool apply_settings;

	extern bool play_pause;

	extern float bounds[3];
	extern int num_spheres;
	extern float sphere_max_r;
	extern float sphere_min_r;

	extern float sim_delta;
} // namespace panel