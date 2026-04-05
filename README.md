
# Implementation Methods
CPSC 587 Assignment 4,
Holden Holzer, holden.holzer@ucalgary.ca
## AI Statement
No AI was used. 
## Spring and Mass Force Calculations
Each spring is modeled as a connection between two masses. There are two forces that the spring imparts of the the masses. First the spring force which modeled as being linearly proportional to the change in length of the spring from the rest length of the spring:
$$ F_s = -k(x - r)$$
Where $F_s$ is the force in [N], $k$ is the spring constant in [N/m], $r$ is the rest length of the spring [m], and x is the difference in position between the ends of the spring [m]. \
If we suppose that the position in 3 dimensions of the two masses at the ends of the spring are $\lbrace P, Q\rbrace$ then the spring force in 3 dimensions is: $$ \vec{F_s} = -k(\lvert\lvert P - Q \rvert\rvert - r) \cdot \frac{P - Q}{\lvert\lvert P - Q\rvert\rvert} $$
The second force to consider is the viscous damping force, this is modeled as being linearly proportional to the speed of extension or contraction of the spring. $$ F_d = -c\frac{dx}{dt}$$
Where $F_d$ is the damping force [N], $c$ is the viscous damping coefficient in [N*s/m], and dx/dt is the velocity of the extension of the spring in [m/s]. Suppose the velocities of the two masses are $\lbrace v_p, v_q\rbrace$ In 3 dimensions this force is:$$\vec{F_d} = -c \frac{(v_p - v_q)\cdot(P - Q)}{\lvert\lvert P - Q\rvert\rvert}\frac{(P - Q)}{\lvert\lvert P - Q\rvert\rvert}$$
The resulting forces on each mass is:$$F_Q = F_s + F_d$$ $$F_P = -F_s - F_d = -F_Q$$
During each iteration of the simulation, the forces from each spring are calculated and added to the forces on each mass.   

## Position Update Method
For this assignment the Semi-implicit Euler method was used to update the positions and velocities of each mass:$$\vec{v}(t+\Delta{t}) = \vec{v}(t) + \frac{\vec{F}}{m}\cdot \Delta{t}  $$ $$\vec{x}(t+\Delta{t}) = \vec{x}(t) + \vec{v}({t+\Delta{t}})\Delta{t}$$
Where $\vec{F}$ is the total force on the mass being updated in [N], $m$ is the mass of the current mass, $\Delta{t}$ is the time step size in [s], $\vec{v}(t)$ is the velocity of the mass at time $t$ in [m/s], $\vec{v}(t + \Delta{t})$ is the velocity of the mass at time $t + \Delta{t}$, $\vec{x}(t)$ is the position of the mass at time $t$ in [m], $\vec{x}(t+\Delta{t})$ is the position of the mass at time $t+\Delta{t}$.
$$$$
## Linear Viscous Air Damping
Because the spring damping only adds damping in the direction of the displacement between the two masses, it does not add damping to motion perpendicular to the spring, nor does it damp bulk motion of the connected masses. Without damping in these directions, the simulation could potentially become unstable. To prevent this we add damping against the motion of the masses. This can be thought of as linear viscous air damping (though forces from air are highly complex and non linear in reality). The force added to each mass is calculated using:$$\vec{F_{air}} = -k_a \cdot \vec{v}$$
Where $\vec{F_{air}}$ is the damping force on the mass in [N], $k_a$ is the air damping coefficient [N*s/m], $\vec{v}$ is the velocity of the mass in [m/s]. 
## Model 1: Mass on a Spring
The mass on spring model is simply two masses connected by one spring in a gravity field, one with fixed position. The motion of the non fixed mass is driven by calculating the spring force on it, adding the "force" of gravity then updating the position and velocity using the Semi-implicit Euler method.
## Chain Pendulum
The Chain Pendulum model is a chain of masses connected by springs in a gravity field. Viscous air damping is also present in this model to prevent the model from swaying indefinitely.  
## Cube of Jelly
The Cube of Jelly is a 3D array of masses in a gravity field, the masses are connected by springs to their neighbors, linear viscous damping is present, Collisions are resolved using the impulse method with friction. \
The connections between masses are setup such that each mass $m_i$ has a spring connecting it to any neighbor within a radius, that is any masses $m_j, m_i$ satisfying $ \lvert\lvert P_i - P_j\rvert\rvert < R$ have a spring between them. It was found to be advantageous to set the radius $R$ large enough such that masses are connected to masses that are located at least 2 indices over, rather than just the immediate neighbors, as this helps to prevent the cube from inverting on itself.  
### Collision Handling
The Cube of jelly must collide with the ground. The interaction between the ground and the cube of jelly was accomplished using the following methods. \
**Collision Detection:** \
Each mass $i$ stores its current position $\vec{x_i}(t)$ and previous position $\vec{x_i}(t + \Delta{t})$. We assume the ground is the $y=0$ plane. During each update, we check each mass. If the current position of the mass is bellow $y=0$ and the previous position was above $y=0$. Then that mass has hit the ground.\
**Collision Resolution:** \
For collision resolution, we use the equations presented in the lecture material "Collisions Slides".
If a collision is detected, we know the collision normal is the $y$ axis $[0,1,0]$. The mass of the ground is assumed to be infinite so the resulting velocity in the $y$ direction simplifies to: $$-e \cdot v_y$$
Where $e$ is the coefficient of restitution, and $v_y$ is the velocity in the y direction. \
The magnitude (direction of the tangential velocity remains unchanged after the collision so only the magnitude is important here) of the new tangential velocity is calculated as: $$V_{t} = max( (v_t - c_f \cdot \Delta{v_y}), 0) $$
Where $V_t$ is the magnitude of the new tangential velocity. $\Delta{v_y}$ is the change in $y$ velocity from the collision. $c_f$ is the coefficient of friction. $v_t$ is the old magnitude of the tangential velocity. \
For simplicity, the position in the y direction is just set to 0 if a collision is detected (this was found to prevent the cube inverting on itself, since vertices were not being shoved deep into the cube in a collision update).   

## Hanging Cloth
The Hanging Cloth model is a 2-D array of masses in a gravity field with springs connecting masses to their neighbors and two corners fixed. The model also includes springs to prevent out of plane bending, and linear air resistance. 
## Flag Model
The flag model is nearly identical to the hanging cloth with one exception. The forces of air on on the model is different:
### Flag Aerodynamic Forces
The faces of the flag are defined as triangles, each triangle having corners defined by 3 masses we will denote as $\brace m_a, m_b, m_c \rbrace$. \
Suppose $\vec{v_i} = \frac{1}{3} \cdot(\vec{v_a} + \vec{v_b} + \vec{v_c})$ is the average velocity of the triangle. \
The velocity of the air is $\vec{U_{air}}$. \
The relative velocity between the air and the face is then: $\vec{v_r} = \vec{U_{air}} - \vec{v_i}$.

The force of air on an object is proportional to: \
(1) The area of the object, in the case of this model we use the cross product rule to calculate the area $A$ of each triangle face as: $$ \vec{d_{ba}} = \vec{P_b} - \vec{P_a}$$
$$\vec{d_{ca}} = \vec{P_c} - \vec{P_a}$$
$$ A = \frac{1}{2}\lvert\lvert d_{ba} \times d_{ca}\rvert\rvert$$
$$ \vec{N} = \frac{d_{ba} \times d_{ca}}{\lvert\lvert d_{ba} \times d_{ca}\rvert\rvert}$$
(2) The specific force is the momentum transported by the air to the object. First we calculate the amount of momentum carrying mass interacting with the surface as being proportional to the density $\rho$ of the air multiplied by the relative speed $v_{r}$ of the air $$\lvert\lvert \vec{v_{r}}\rvert\rvert * \rho$$
(3) The amount of momentum each piece of mass of air deposits on the object (The change in velocity of the air). Here we split the forces into a normal and tangent component, since the velocity normal to the surface will be changed by much more than the velocity tangent to a surface: $$\vec{V_n} = (\vec{V_r}\cdot \vec{N})\vec{N}$$ $$\vec{V_t} = \vec{V_r} - \vec{V_n}$$
The factors by which the velocity changes in each of these directions will be denoted as $C_{Dn}$ for the normal direction, and $C_{Dt}$ for the tangent direction.\
Combining these factors gives the forces as: $$\vec{F_n} = C_{Dn} \rho A \lvert\lvert \vec{v_{r}}\rvert\rvert \vec{V_n}$$ $$\vec{F_t} = C_{Dt} \rho A \lvert\lvert \vec{v_{r}}\rvert\rvert \vec{V_t}$$
For simplicity. By basic intuition of how air works, we will assume that the air nearly stops when it hits in the normal direction, but continues to slide along the surface in the tangent direction. So we chose a $C_{Dn}$ that is much larger than $C_{Dt}$. \
If this model is an accurate enough model of the fluid surface interaction, then the flag should naturally oscillate when the wind speed is high enough (We have intentionally not added forced turbulence, as this model gives a much more natural look to the waving motion of the flag)        
## Cloth Falling on Table
The cloth falling on table model uses the hanging cloth combined with the collision resolution system from the Cube of Jelly. The ground is modified to only detect collisions within a radius R of the origin. This produces the collision of a circular table top.
## Inextensible Chain
The Inextensible Chain is a modification of the Chain Pendulum Model. Instead of connecting the masses with springs, the DFTL algorithm from [Müller M, Kim TY, Chentanez N. Fast Simulation of Inextensible Hair and Fur. VRIPHYS. 2012 May;12:39-44.] is used to update the masses. The algorithm is executed as follows.

1. Calculate an initial estimate of for the new positions of the masses based on gravity and linear air damping forces:$$\vec{x_i}(t+\Delta{t}) \leftarrow \vec{x_i}(t) + \vec{v}({t})\Delta{t} + \vec{a}*\Delta{t}^2$$
2. calculate FTL (Follow the leader) correction: $$\vec{d_i} = (p_i - p_{i-1}) \left( \frac{R}{\lvert\lvert p_i - p_{i-1} \rvert\rvert} - 1\right)$$
3. calculate the new position:$$\vec{x_i}(t+\Delta{t}) \leftarrow \vec{x_i}(t+\Delta{t}) + \vec{d_i}$$
4. calculate the velocity correction:$$\vec{v_i}(t+\Delta{t}) = \frac{\vec{x_i}(t+\Delta{t}) - \vec{x_i}(t)- s_{damping} \cdot \vec{d_{i-1}}}{\Delta{t}}$$

In testing this solution is stable for values of $s_{damping}$ very near 1.
# Usage
## Building and Running
To quickly build and run the program it is advised to run the provided shell scripts using the command:
"./QuickBuild.sh" or "./CleanBuild.sh" these will build and run the program in a single command. \
**Building**: To build the program navigate to the directory containing "src", "models", "libs", "CMakeLists.txt". Run the command "cmake -B build", then run the command "cmake --build build". The executable will be named "cpsc587_a4_hh" \
**Running**: Run the command "./build/cpsc587_a4_hh" 
## Controls
* **Show Wireframe:** This input is available in the "Cube of Jelly", "Hanging Cloth", "Flag in Wind", and "Table Cloth" models, it switches the view to show the springs and masses in the model.
* **Show Masses:** In the "Inextensible Rope" model, this input shows the locations of masses on the rope.
* **Wind Velocity:** In the "Flag in Wind" model, this input allows the user to control the direction of and speed of the wind hitting the flag. 
* **Reset View:** This input remains unchanged from the example code.
* **Iterations Per Frame** This input remains unchanged from the example code.
* **Model:** This input remains unchanged from the example code.
* **Play Simulation:** This input remains unchanged from the example code.
* **Reset Simulation:** This input remains unchanged from the example code.
* **Step Simulation:** This input remains unchanged from the example code.
* **Simulation dt (s):** This input remains unchanged from the example code.