
# Implementation Methods
CPSC 587 Assignment 4,
Holden Holzer, holden.holzer@ucalgary.ca
## AI Statement
No AI was used. 
## The Three Boid Forces
I you imagine that the forces are coming from air (like how a flock of birds impart forces on the air not eachother) then the Boid forces could be satisfying Newtons Third law.
### Separation
The separation force is used to push boids away from each other if they get too close. We define the distance where this force starts as $r_s$ the radius of separation. Since the Boid is intended to model a flocking animal, we also only want it to come into effect if the boid to avoid (we will denote as $b_j$) is in the field of view of the current boid (denote as $b_i$). The field of view will be defined by the angle $\alpha_s$. In this simulation the force was made proportional to the inverse of the distance between the two boids. The calculation of the separation force is as follows:
1. If the distance between the two boids is within the detection distance $\lvert\lvert p_j - p_i\rvert\rvert < r_s$ then:
2. If it is in the field of view: $$cos^{-1}\left(\frac{(p_j - p_i) \cdot v_i}{\lvert\lvert p_j - p_i\rvert\rvert \lvert\lvert v_i\rvert\rvert}\right) < \alpha_s$$
Note here that we assume boid i is face in its direction of motion $v_i$ (In the simulation, $cos(\alpha_s)$ is pre calculated to avoid the use of trigonometric functions.
3. Within the detection distance and field of view then the force/acceleration is:$$a^s_{ij} = -k_s \cdot \frac{(p_j - p_i)}{\lvert\lvert p_j - p_i\rvert\rvert} \frac{1}{\lvert\lvert p_j - p_i\rvert\rvert}$$
Where $k_s$ is the separation force constant. 
### Alignment
The alignment force is used to make boids move in the same direction. Like with the separation force, this has an associated detection distance and view angle $r_{align}$ and $\alpha_{align}$ In this simulation the force was made proportional to the velocity difference between the two boids. The calculation of the alignment force is as follows:
1. If the distance between the two boids is within the detection distance $\lvert\lvert p_j - p_i\rvert\rvert < r_{align}$ then:
2. If it is in the field of view: $$cos^{-1}\left(\frac{(p_j - p_i) \cdot v_i}{\lvert\lvert p_j - p_i\rvert\rvert \lvert\lvert v_i\rvert\rvert}\right) < \alpha_{align}$$
Note here that we assume boid i is face in its direction of motion $v_i$ (In the simulation, $cos(\alpha_{align})$ is pre calculated to avoid the use of trigonometric functions.
3. Within the detection distance and field of view then the force/acceleration is:$$a^{align}_{ij} = -k_a \cdot (v_j - v_i) $$
Where $k_a$ is the alignment force constant. 
### Cohesion
The Cohesion force is used to make boids group together. Like with the separation force, this has an associated detection distance and view angle $r_{c}$ and $\alpha_{c}$ In this simulation the force was made proportional to the distance between the two boids. The calculation of the alignment force is as follows:
1. If the distance between the two boids is within the detection distance $\lvert\lvert p_j - p_i\rvert\rvert < r_{c}$ then:
2. If it is in the field of view: $$cos^{-1}\left(\frac{(p_j - p_i) \cdot v_i}{\lvert\lvert p_j - p_i\rvert\rvert \lvert\lvert v_i\rvert\rvert}\right) < \alpha_{c}$$
Note here that we assume boid i is face in its direction of motion $v_i$ (In the simulation, $cos(\alpha_{c})$ is pre calculated to avoid the use of trigonometric functions.
3. Within the detection distance and field of view then the force/acceleration is:$$a^c_{ij} = -k_c \cdot (p_j - p_i) $$
Where $k_c$ is the cohesion force constant. 
## Combining these forces
The forces are combined in the following ways: 
### Ordering:
The forces are applied such that for each neighbor $b_j$ of boid $b_i$. Only one force type is applied at a time. The forces use the detection distances to determine which force to apply
1. **if** $r_{align} < \lvert\lvert p_j - p_i\rvert\rvert < r_{c}$ then apply cohesion force, otherwise:
2. **if** $r_s < \lvert\lvert p_j - p_i\rvert\rvert < r_{align}$ then apply alignment force, otherwise:
3. **if** $\lvert\lvert p_j - p_i\rvert\rvert < r_s$ then apply separation force.

This method prevents the forces from "fighting" each other.
### normalizing
A boid may have an arbitrary number of neighbors. To avoid having to adjust the force constants depending on the number of neighbors, the forces/accelerations are normalized by the number of neighbors:$$a_{i}^{total} = \frac{1}{N}\sum_{j=0}^N (a^s_{ij} + a^{align}_{ij} + a^c_{ij})$$
Where N is the number of neighbors of boid i.
### clamping the forces
Finally, to ensure the acceleration is reasonable it is clamped between zero and some maximum acceleration. This also handles the case of NAN or Infinite values.
## Position Integration Method:
For this assignment the Semi-implicit Euler method was used to update the positions and velocities of each boid:$$\vec{v}(t+\Delta{t}) = \vec{v}(t) + a_{i}^{total} \Delta{t}  $$ $$\vec{x}(t+\Delta{t}) = \vec{x}(t) + \vec{v}({t+\Delta{t}})\Delta{t}$$
Where $\Delta{t}$ is the time step size in [s], $\vec{v}(t)$ is the velocity of the boid at time $t$ in [m/s], $\vec{v}(t + \Delta{t})$ is the velocity of the boid at time $t + \Delta{t}$, $\vec{x}(t)$ is the position of the boid at time $t$ in [m], $\vec{x}(t+\Delta{t})$ is the position of the boid at time $t+\Delta{t}$.\
In the simulation, the velocity magnitude is clamped to some maximum speed value. The position is clamped between the bounds of the simulation volume.
$$$$
## Steering around infinite Planes
Avoiding planes is done using two methods. (1) steering for when the boid is far from the plane, (2) repulsion force if the boid gets too close to the plane.
### Repulsion
A plane has an offset distance $\epsilon$. If the boid gets within this distance of the plane, then repulsion force is used. For this simulation, the repulsion force is just set to some maximum allowed acceleration to prevent the boid from impacting the plane. The acceleration is calculated as follows: $$a_{repulsion} = k_{max} \cdot \hat{n}$$
Where $k_{max}$ is the maximum acceleration factor, and $\hat{n}$ is the normal of the plane.
### Steering
The steering method for avoiding planes is identical to that outlined in "Assignment 5 - Technical Specifications". The basic algorithm is as follows:
1. Check if the plane is within the detection radius of the boid, we use the maximum of the three radii for detecting if the plane is in the detection range of the boid $max(r_{a}, r_{align}, r_{c})$.
2. Check if the boid is current pointed towards the plane using the velocity vector dotted with the normal to determine if it is facing the plane $v_i \cdot \hat{n} < 0$
3. Check if it is within the offset of the plane, if so implement the repulsion force
4. Calculate the acceleration direction as the component of the plane normal that is tangent to the velocity vector: $$\hat{a}_c = \frac{\hat{n} - (\hat{n}\cdot\hat{v})\hat{v}}{\lvert\lvert\hat{n} - (\hat{n}\cdot\hat{v})\hat{v}\rvert\rvert}$$
5. Use equation for the radius given by "Assignment 5 - Technical Specifications" to calculate the steering radius.$$r = \frac{(P-C)\cdot \hat{n} - \epsilon}{1 - \hat{a}_c\cdot\hat{n}}$$
6. Use the steering radius and current velocity to calculate the required acceleration: $$\vec{a}_c = \frac{\lvert\lvert\vec{v}\rvert\rvert^2}{r}\hat{a}_c$$
In the simulation implementation, checks are made to avoid the scenario where the boid is pointed directly at the plane normal, and therefore does not have a valid acceleration direction. To avoid this a random vector is given as the acceleration direction.
## Steering around Spheres
Avoiding Spheres is done using the same two methods as planes. (1) steering for when the boid is far from the sphere, (2) repulsion force if the boid gets too close to the sphere.
### Repulsion
A sphere has an offset distance $\epsilon$. If the boid gets within this distance of the sphere, then repulsion force is used. For this simulation, the repulsion force is just set to some maximum allowed acceleration to prevent the boid from impacting the sphere. The acceleration is calculated as follows: $$a_{repulsion} = k_{max} \cdot (P-C)$$
Where $k_{max}$ is the maximum acceleration factor, and $P-C$ is direction vector from the sphere center to the boid.
### Steering
The steering method for avoiding sphere is identical to that outlined in "Assignment 5 - Technical Specifications". The basic algorithm is as follows:
1. Check if the sphere is within the detection radius of the boid, we use the maximum of the three radii for detecting if the sphere is in the detection range of the boid $max(r_{a}, r_{align}, r_{c})$.
2. Check if the boid is current pointed away from the sphere using the velocity vector dotted with the displacement $P-C$ to determine if it is facing the sphere $v_i \cdot (P-C) < 0$
3. Check to see if the trajectory intersects the sphere using the identity:$$ b^2 = \lvert\lvert P-C \rvert\rvert^2 - (\vec{v}\cdot(P-C))^2$$ Where if $b$ is greater than $R+\epsilon$ ($R$ being the sphere radius) then the boid will miss the sphere. 
4. Check if it is within the offset of the sphere, if so implement the repulsion force
5. Calculate the acceleration direction as the component of the plane normal that is tangent to the velocity vector: $$\hat{a}_c = \frac{\hat{v} - (\hat{v}\cdot(P-C))(P-C)}{\lvert\lvert \hat{v} - (\hat{v}\cdot(P-C))(P-C) \rvert\rvert}$$
6. Use equation for the radius given by "Assignment 5 - Technical Specifications" to calculate the steering radius.$$r = \frac{\lvert\lvert(P-C)\rvert\rvert^2 - (R+\epsilon)^2}{2 ((R+\epsilon) - (P-C)\cdot \hat{a}_c)}$$
7. Use the steering radius and current velocity to calculate the required acceleration: $$\vec{a}_c = \frac{\lvert\lvert\vec{v}\rvert\rvert^2}{r}\hat{a}_c$$
In the simulation implementation, checks are made to avoid the scenario where the boid is pointed directly at the sphere center, and therefore does not have a valid acceleration direction. To avoid this a random vector is given as the acceleration direction.
## Orienting Boids
The rotation of the boids are calculated using the frenet frame. First the tangent vector is calculated as the normalized velocity vector $T = v / \lvert\lvert v \rvert\rvert$. The Normal is calculated by finding total acceleration vector and taking its component that is perpendicular to the velocity. The acceleration due to gravity is added to get the total acceleration: $\vec{a} = \vec{a}_{raw} - \vec{g}$. Then we get the component of this acceleration that is perpendicular to the velocity $\vec{a}_{perp} = \vec{a} - (\vec{a} \cdot \vec{T})\vec{T}$. This vector is normalized to get the normal for the rotation matrix $N = \vec{a}_{perp} / ||\vec{a}_{perp}||$. This can then be used to get the Binormal $B = N \times T$. These vectors then form the rotation matrix used to orient the boid.
## Detecting Neighbors: Uniform grid for the broad phase
The simulation uses a uniform grid for the broad phase in finding potential neighbor boids. This Method divides the grid into regularly spaces cubic cells of side length $c$. In our simulation, c is set to the detection radius for cohesion $r_c$ as this means that a neighbor can be at most 1 cell away from the current boid while minimizing the cell size and therefore number of boids in each cell. The cell that a boid belongs to is found by taking the position of a boid and dividing it by the cube size. The cell index is found by: $$x_{index} = \lfloor{x_{boid} - O_x/ c}\rfloor$$
$$y_{index} = \lfloor{y_{boid} - O_y/ c}\rfloor$$
$$z_{index} = \lfloor{z_{boid} - O_z/ c}\rfloor$$
$$ cell_{index} = x_{index}N_yN_z + y_{index}N_z + z_{index}$$
Where $N_x,N_y$ are the number of cells in the x and y directions. $O_x, O_y, O_z$ is the origin of the grid. $x_{boid}, y_{boid}, z_{boid}$ is the position of the boid.
This grid is used in the following way: after every update, each boid is hashed and added to the boid list of its cell, along with the 26 neighboring cells. During the update, each cell only needs to consider the boids in its cell since its cells boid list also contains the boids that are in the 26 neighboring cells. 
## Bonus 3 Entry: Simulation Speed Up Tricks
I Opt-in to the competition for bonus 3
### Multi-Threaded Simulation
The force calculation, velocity update, and position update is multi-threaded. The boids are equally split among a given number of threads. 
### Rendering thread Running parallel to the simulation
The task of calculating the orientation matrix and adding the render instance for each boid is handled by a set of rendering threads. These threads run in parallel to the Simulation threads. This is accomplished by having a 3-buffer array of boids. One boid array is updated by the simulation threads, A second boid array holds the result of the most recent complete simulation step, and the Third boid array holds results of the second most recent simulation step. These buffers are swapped when a simulation step is completed, the oldest values being assigned as the new array for updating. This system allows the render threads to read from the results and draw the boids while the simulation updates a new batch of results in parallel. 
### Max Neighbors
To avoid the scenario of calculating the boid forces of an extremely large number of neighbors, we place a maximum number of neighbors to consider after which no new boid forces are added in a given simulation step. This is based on the assumption that if a boid is surrounded by a large number of neighbors, then the average direction of the force form these neighbors can be approximated by a random sampling of a subset of neighbors. Also, since the force is normalize by the number of neighbors, this also should not effect the magnitude of the force by much.
### Position Super Sampling
Because the simulation uses three boid data buffers, two of which store the results of the last two most recent simulation steps. This allows the renderer to update at a faster updated rate than the simulation and simply interpolate values between the two most recent results Example $$p(t) = \frac{t-t_{start}}{t_{end} - t_{start}}(p_{end} - p_{start}) + p_{start}$$
The accelerations, velocities, and positions are also low pass filtered to prevent sudden changes in orientation when the buffers are swaped.
# Usage
## Building and Running
To quickly build and run the program it is advised to run the provided shell scripts using the command:
"./QuickBuild.sh" or "./CleanBuild.sh" these will build and run the program in a single command. \
**Building**: To build the program navigate to the directory containing "src", "models", "libs", "CMakeLists.txt". Run the command "cmake -B build", then run the command "cmake --build build". The executable will be named "cpsc587_a5_hh" \
**Running**: Run the command "./build/cpsc587_a5_hh" 
## Controls
* **Play Simulaiton** Starts the simulation.
* **Reset Simulation** Resets the boid positions to random positions.

**!!NOTICE!!** The following controls do not take effect unless **Apply Settings** is pressed, this is because most variables effect the data structures of the simulation and require restarting the simulation.
* **Number of boids** Set the number of boids in the simulation
* **Angle detection separation** The field of view angle for separation
* **Angle detection Alignment** The field of view angle for alignment
* **Angle detection cohesion** The field of view angle for cohesion
* **Radius separation** The detection radius for separation
* **Radius Alignment** The detection radius for alignment
* **Radius cohesion** The detection radius for cohesion
* **k separation** The force constant for separation
* **k alignment** The force constant for alignment
* **k cohesion** The force constant for cohesion
* **surface offset** set the surface offset constant (for both planes and spheres)
* **Max speed** the maximum speed bound for the boids
* **Max acceleration** the maximum acceleration bound for the boids
* **bounds** The bounds of the simulation volume (not that the simulation volume is from -bound to +bound)
* **Number of spheres** The number of spheres
* **Sphere max radius** The maximum sphere radius
* **Sphere min radius** The minimum sphere radius
* **Apply settings** Restarts the simulation using all the settings above
* **Setup Scenario 100,000** Pre sets up a scenario with 100,000 boids
* **Setup Scenario EXTREME** Pre sets up the scenario for the competition
