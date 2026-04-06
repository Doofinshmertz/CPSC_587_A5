#include "imgui_panel.hpp"

namespace imgui_panel {
	// default values
	bool showPanel = true;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	bool reset_view = false;

	//Simulation settings
	int number_of_iterations_per_frame = 1;
	bool play_simulation = false;
	bool reset_simulation = false;
	bool step_simulation = false;
	float dt_simulation = 0.001f;
	bool enable_validity_check = true;
	int n_boids = 1500;

	// Simulation State
	bool valid_simulation = true;
	float t_simulation = 0.f;
	int n_frames = 0;
	int n_frames_less_than_20 = 0;

	// custom values
	float angle_sep = 175.0f;
	float angle_align = 140.0f;
	float angle_coh = 120.0f;

	float r_sep = 6.0f;
	float r_align = 8.0f;
	float r_coh = 10.0f;

	float k_sep = 10.0f;
	float k_align = 2.0f;
	float k_coh = 0.4f;
	
	float offset = 1.0f;
	float k_repulsion = 3.0f;

	float max_speed = 20.0f;
	float max_acceleration = 100.0f;

	bool apply_settings = false;
	bool play_pause = false;

	float sim_delta = 0.0f;

	float bounds[3] = {100.0f, 100.0f, 100.0f};
	int num_spheres = 14;
	float sphere_max_r = 10.0f;
	float sphere_min_r = 4.0f;

	std::function<void(void)> draw = [](void) {
		if (showPanel) {
			if (ImGui::Begin("Panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
				ImGui::Spacing();
				ImGui::Separator();

				ImGui::ColorEdit3("Clear color", (float*)&clear_color);
				reset_view = ImGui::Button("Reset View");

				ImGui::Spacing();
				ImGui::Separator();

				ImGui::SliderInt("Iterations Per Frame", &number_of_iterations_per_frame, 1, 100);
				if(ImGui::Checkbox("Play Simulation", &play_simulation))
				{
					play_pause = true;
				}
				reset_simulation = ImGui::Button("Reset Simulation");
				if (!play_simulation) {
					step_simulation = ImGui::Button("Step Simulation");
				}
				ImGui::DragFloat("Simulation dt (s)", &dt_simulation, 1.e-5f, 1.e-6f, 1.f, "%.6e");
				ImGui::Checkbox("Validity Check (potentially slow)", &enable_validity_check);

				ImGui::Spacing();
				ImGui::Separator();

				ImGui::InputInt("Number of boids", &n_boids);
				ImGui::Spacing();
				
				// detection angle inputs
				ImGui::DragFloat("Angle detection separation", &angle_sep, 0.01f, 0.0f, 179.0f);
				ImGui::DragFloat("Angle detection Alignment", &angle_align, 0.01f, 0.0f, 179.0f);
				ImGui::DragFloat("Angle detection cohesion", &angle_coh, 0.01f, 0.0f, 179.0f);
				ImGui::Spacing();

				// detection radius inputs
				ImGui::DragFloat("Radius separation", &r_sep, 0.01f, 0.0f, 10.0f);
				ImGui::DragFloat("Radius Alignment", &r_align, 0.01f, 0.0f, 10.0f);
				ImGui::DragFloat("Radius cohesion", &r_coh, 0.01f, 0.0f, 10.0f);
				ImGui::Spacing();

				// acceleration factors
				ImGui::DragFloat("K separation", &k_sep, 0.01f, 0.0f, 100.0f);
				ImGui::DragFloat("K alignment", &k_align, 0.01f, 0.0f, 10.0f);
				ImGui::DragFloat("K cohesion", &k_coh, 0.01f, 0.0f, 10.0f);
				ImGui::Spacing();

				ImGui::DragFloat("surface offset distance", &offset, 0.01f, 0.0f, 100.0f);
				ImGui::DragFloat("k repulsion", &k_repulsion, 0.01f, 0.0f, 10.0f);
				ImGui::Spacing();

				ImGui::DragFloat("Max speed", &max_speed, 0.1f, 1.0f, 100.0f);
				ImGui::DragFloat("Max acceleration", &max_acceleration, 0.1f, 1.0f, 1000.0f);

				ImGui::Spacing();
				if(ImGui::InputFloat3("bounds", bounds))
				{
					for(int i = 0; i < 3; i++)
					{
						if(bounds[i] < 5.0f)
						{
							bounds[i] = 5.0f;
						}
						else if(bounds[i] > 200.0f)
						{
							bounds[i] = 200.0f;
						}
					}
				}
				if(ImGui::InputInt("Number of spheres", &num_spheres))
				{
					if(num_spheres < 14)
					{
						num_spheres = 14;
					}
					else if(num_spheres > 100)
					{
						num_spheres = 100;
					}
				}

				ImGui::DragFloat("Sphere max Radius", &sphere_max_r, 0.1f, 1.0f, 100.0f);
				ImGui::DragFloat("Sphere min Radius", &sphere_min_r, 0.1f, 1.0f, 100.0f);
				

				apply_settings = ImGui::Button("Apply Settings");
				if(apply_settings)
				{
					if(sphere_max_r < sphere_min_r)
					{
						sphere_max_r =20.0f;
						sphere_min_r = 5.0f;
					}
				}
				
				ImGui::Spacing();
				ImGui::Separator();

				ImGui::Text("Simulation Time: %.6f s", t_simulation);
				ImGui::Text("Is simulation valid? %s", enable_validity_check ? valid_simulation ? "YES" : "NO" : "Vibes-based (Disabled)");

				float percent_of_frames_less_than_20 = 100.f * (
					n_frames == 0ULL ? 0.f : static_cast<float>(n_frames_less_than_20) / static_cast<float>(n_frames)
					);
				ImGui::Text("Percent of Frames less than 20 FPS: %.3f", percent_of_frames_less_than_20);

				ImGui::Spacing();
				ImGui::Separator();

				float frame_rate = ImGui::GetIO().Framerate;
				ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
					1000.0f / frame_rate, frame_rate);

				ImGui::Text("Simulation timestep: %.3f s/frame (%.1f FPS)", sim_delta, (1.0f / sim_delta));

				ImGui::Spacing();
				ImGui::Separator();
			}
			ImGui::End();
		}
	};
} // namespace panel