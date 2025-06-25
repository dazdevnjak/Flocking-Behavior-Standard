# Flocking & Predator-Prey Simulation

A Unity 3D project demonstrating a Boids-style flocking system with predator-prey interactions, obstacle avoidance, and world-bounds containment.

---

## 🚀 Features

- **Classic Boids**  
  - **Separation**: avoid crowding neighbors  
  - **Alignment**: match heading with flockmates  
  - **Cohesion**: steer toward flock’s center of mass  

- **Predator–Prey Dynamics**  
  - **Pursuit**: predators predict and chase prey  
  - **Evasion**: prey predict and flee from predators  
  - Configurable **attack duration** & **cooldown**

- **Auxiliary Behaviors**  
  - **Wander** via Perlin noise  
  - **Obstacle Avoidance**: sampled raycasts find clear paths  
  - **Bounds Containment**: keeps boids inside a cubic region  
  - **Noise**: small perturbations for natural motion  

- **Smooth Integration**  
  - Velocity clamped to configurable `maxSpeed`  
  - Lerp-based smoothing for fluid acceleration & turning  
  - Adjustable weights for each steering component  

---

https://github.com/user-attachments/assets/19601446-49a3-4a2e-8db8-dca8c958404b
