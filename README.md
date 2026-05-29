# Drone Swarm Combat Simulator

A fully autonomous, multi-agent drone combat simulator built entirely in MATLAB. This project demonstrates advanced robotics concepts, including swarm intelligence, dynamic collision avoidance, and path planning, all running in a high-speed, custom 2D physics engine.

## 🚀 Features

*   **Global Target Assignment (Swarm Intelligence):** Uses a DeepSORT proxy and the Hungarian algorithm to dynamically negotiate and assign targets across the swarm network. If an enemy enters a drone's sensor range, the swarm instantly calculates the optimal interceptor.
*   **Dynamic Collision Avoidance:** Real-time, short-range collision avoidance engine dodging both static obstacles and high-speed moving hazards.
*   **Global Pathfinding:** Charts complex routes through dense obstacle fields to return safely to spawn points.
*   **Visual Servoing:** Advanced control law that smoothly adjusts the drone's velocity and bearing based on distance to the target, allowing for aggressive high-speed chases and soft, precise landings.
*   **Cinematic Video Recording:** Automatically captures the entire simulation frame-by-frame and exports it as a universally compatible `.avi` video file.

## 🧠 Algorithms Under the Hood

This project was built from scratch to implement and demonstrate the following state-of-the-art robotics and computer vision algorithms:

*   **YOLO (Detection):** Simulates a YOLO (You Only Look Once) neural network pipeline. When an enemy breaches the 45-meter sensor bubble, the simulated YOLO sensor flags the detection and adds realistic Gaussian noise to emulate real-world camera inaccuracies.
*   **Triangulation (Positioning):** Takes the raw, noisy bearing and range data from the YOLO detection and calculates the target's absolute 2D coordinate on the battlefield.
*   **DeepSORT (Tracking & ID):** A proxy algorithm that assigns persistent, unique IDs to enemy drones across multiple frames, ensuring that the swarm doesn't lose track of a target even if the sensor feed temporarily drops.
*   **Kalman Filter (Motion Prediction):** Continuously predicts the future position and velocity of the enemy targets. It smooths out the noisy YOLO detections, allowing the friendly drones to aim where the enemy *will* be, rather than where they *were*.
*   **Hungarian Algorithm (Assignment):** The core of the swarm intelligence. It takes the matrix of all friendly drones and all tracked enemies, and calculates the mathematically optimal 1-to-1 target assignment to minimize the total global flight distance of the swarm.
*   **RRT* (Rapidly-exploring Random Tree Star):** The global path generation algorithm. When a drone needs to return home, RRT* rapidly branches out thousands of possible paths through the dense static obstacle field to find a safe, collision-free route.
*   **DWA (Dynamic Window Approach):** The short-range, real-time obstacle avoidance system. It simulates hundreds of possible velocities and steering angles for the next 1.5 seconds, scoring them based on target heading and obstacle clearance to weave through tight gaps at high speed.
*   **ORCA (Optimal Reciprocal Collision Avoidance):** The inter-drone safety protocol. It calculates "velocity obstacles" to ensure that friendly drones never crash into each other when crossing paths or converging on the same area.
*   **Visual Servoing:** A proportional control law that acts as the drone's throttle. It calculates the exact speed and turning radius required based on the distance to the goal—allowing for blazing-fast 12m/s pursuits, and smooth, decelerating 0m/s landings.

## ⚙️ How it Works

1.  **Seek Mode:** Friendly drones (blue) start with a limited sensor range and blindly push forward into the battlefield.
2.  **Detect & Assign:** When an enemy drone (red) breaches the sensor bubble, the YOLO/Kalman pipeline tracks it. The Hungarian algorithm dynamically assigns the closest friendly drone, changing the enemy's label to match its hunter (e.g., "Target 3").
3.  **Pursuit & Evasion:** The assigned drone kicks into high gear, using DWA and ORCA to weave through asteroid-like obstacle fields without crashing.
4.  **Capture & Return:** Once the intercept distance is closed, the enemy is marked as "captured". The friendly drone immediately runs the RRT* pathfinder to chart a safe route back.
5.  **Soft Landing:** Visual Servoing seamlessly takes over from the high-speed pursuit, gently decelerating the drone to park perfectly on its launch pad.

## 🛠️ Usage

1.  Open `drone_combat_sim.m` in MATLAB or MATLAB Online.
2.  Run the script.
3.  Watch the swarm intelligently negotiate targets and dodge obstacles!
4.  When the simulation completes, locate the newly generated `drone_swarm_combat.avi` file in your Current Folder to view or share the recorded battle.
