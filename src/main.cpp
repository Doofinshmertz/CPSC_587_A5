/**
 * CPSC 587 W26 Assignment 5
 * @name Holden Holzer
 * @email holden.holzer@ucalgary.ca
 *
 * Modified from provided Assignment 5 - Boilerplate
 * @authors Copyright 2019 Lakin Wecker, Jeremy Hart, Andrew Owens and Others (see AUTHORS)
 */


#include "givio.h"
#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp>

#include <panel.h>
#include <picking_controls.h>
#include <turntable_controls.h>

#include "models.hpp"
#include "imgui_panel.hpp"

using namespace giv;
using namespace giv::io;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;


// This is a high performance application, use release mode as the official 
// build type when not debugging. I will be using release mode when grading :). 
// 
// *** Visual Studio users: 
// Your CMake step should automatically detect the configs defined in "CMakeSettings.json".
// You can change between the two above under drop down (x64-Debug vs x64-Release). 
// If you have a 32-bit system (x86), you can add a similar release version. 
// 
// *** Linux (and general) users:
// You have to specify it similar to the README.md compiler flag "-DCMAKE_BUILD_TYPE=Release".

// program entry point
int main(void) {
	// initialize OpenGL and window
	GLFWContext glContext;
	glContext.glMajorVesion(3)
		.glMinorVesion(3)
		.glForwardComaptability(true)
		.glCoreProfile()
		.glAntiAliasingSamples(2)
		.matchPrimaryMonitorVideoMode();
	std::cout << glfwVersionString() << '\n';

	// setup window (OpenGL context)
	ImGuiWindow window = glContext.makeImGuiWindow(Properties()
		.size(dimensions{ 1000, 1000 })
		.title("Boids boids boids!")
		.glslVersionString("#version 330 core"));
	// set our imgui update function
	panel::update_lambda_function = imgui_panel::draw;

	ViewContext view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);


	//  Custom Bind keys
	auto toggle_panel_routine = [&](auto event) {
		if (event.action == GLFW_PRESS)
			imgui_panel::showPanel = !imgui_panel::showPanel;
		};
	auto reset_view_routine = [&](auto event) {view.camera.reset(); };
	auto close_window_routine = [&](auto) { window.shouldClose(); };
	window.keyboardCommands()
		| Key(GLFW_KEY_P, toggle_panel_routine)
		| Key(GLFW_KEY_V, reset_view_routine)
		| Key(GLFW_KEY_ESCAPE, close_window_routine);


	// Models
	std::unique_ptr<BoidSimulation> model = std::make_unique<BoidSimulation>();
	// update the settings on the environment
	// update the settings on the environment
	model->SetSimulationParameters(				
		imgui_panel::angle_sep
		,imgui_panel::angle_align
		,imgui_panel::angle_coh
		,imgui_panel::r_sep
		,imgui_panel::r_align
		,imgui_panel::r_coh
		,imgui_panel::k_sep
		,imgui_panel::k_align
		,imgui_panel::k_sep
		,imgui_panel::offset
		,imgui_panel::k_repulsion
		,imgui_panel::max_speed
		,imgui_panel::max_acceleration
		,imgui_panel::bounds
		,imgui_panel::num_spheres
		,imgui_panel::sphere_max_r
		,imgui_panel::sphere_min_r
		,imgui_panel::n_boids);

	view.camera.zoom(300.0f);
	// main loop
	mainloop(std::move(window), [&](
		float /*dt - Time since last frame. 
			  You should start by using imgui_panel::dt 
			  and only use this under the "Free the Physics" 
			  time step scheme (not required) */
	) {
		// updates from panel
		if (imgui_panel::reset_view) {
			view.camera.reset();
			view.camera.zoom(300.0f);
		}

		if(imgui_panel::apply_settings)
		{
			view.camera.reset();
			view.camera.zoom(imgui_panel::bounds[1]*3.0f);
			// update the settings on the environment
			model->SetSimulationParameters(				
				imgui_panel::angle_sep
				,imgui_panel::angle_align
				,imgui_panel::angle_coh
				,imgui_panel::r_sep
				,imgui_panel::r_align
				,imgui_panel::r_coh
				,imgui_panel::k_sep
				,imgui_panel::k_align
				,imgui_panel::k_sep
				,imgui_panel::offset
				,imgui_panel::k_repulsion
				,imgui_panel::max_speed
				,imgui_panel::max_acceleration
				,imgui_panel::bounds
				,imgui_panel::num_spheres
				,imgui_panel::sphere_max_r
				,imgui_panel::sphere_min_r
				,imgui_panel::n_boids);
		}

		//Simulation updates
		if (imgui_panel::reset_simulation) {
			model->reset();
			imgui_panel::t_simulation = 0.f;
			imgui_panel::n_frames = 0;
			imgui_panel::n_frames_less_than_20 = 0;
		}

		if (imgui_panel::step_simulation) {
			model->step(imgui_panel::dt_simulation);
			imgui_panel::t_simulation += imgui_panel::dt_simulation;
		}

		if(imgui_panel::play_pause)
		{
			imgui_panel::play_pause = false;

			if(imgui_panel::play_simulation)
			{
				model->playSimulation();
			}
			else
			{
				model->pauseSimulation();
			}
		}
		if (imgui_panel::play_simulation) {
			for (size_t i = 0; i < imgui_panel::number_of_iterations_per_frame; i++) {
				model->step(imgui_panel::dt_simulation);
				imgui_panel::t_simulation += imgui_panel::dt_simulation;
			}

			imgui_panel::sim_delta = model->GetDeltatime();
		}

		if (imgui_panel::enable_validity_check) {
			imgui_panel::valid_simulation = model->isValid();
		}

		if (imgui_panel::play_simulation || imgui_panel::step_simulation) {
			imgui_panel::n_frames++;
			if (model->GetDeltatime() > 0.05f) { // Equivalent to < 20 FPS
				imgui_panel::n_frames_less_than_20++;
			}
		}

		// render
		auto color = imgui_panel::clear_color;
		glClearColor(color.x, color.y, color.z, color.w);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		view.projection.updateAspectRatio(window.width(), window.height());

		model->render(view);
	});

	return EXIT_SUCCESS;
}