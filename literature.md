# Algorithms to use

## Odometry

Be robust and able to correct for wheel slip etc.


## Path planning

There is global planning and local planning. For global planning ideally we want a smooth path and
want to be able to run the global planner online, in case of big changes to the environment. That also
means we need to the global planner map to be easily updated.

For local planning we want some kind of reactive type planner that can take into account dynamic
obstacles. This planner should be fast and constantly running

Finally the planned path should be able to handle multiple waypoints and tasks and line up the robot
such that it can easily move to the next task.

* [2013] Safe Mobile Robot Motion Planning for Waypoint Sequences in a Dynamic Environment
  * Dynamic obstacle avoidance part of the algorithm is interesting
  * Also able to move through small gaps
* [2020] Dynamic Local Laplacian Potential Field for UAV Navigation in Unknown Environments
  * Potential based with no local minima. Gives a smooth trajectory
* [2019] Intent-based Robotic Path-Replanning - When to Adapt New Paths in Dynamic Environments
  * How to decide when to replan
* [2020] A 3D Reactive Navigation Algorithm for Mobile Robots by Using Tentacle-Based Sampling
  * Has some nice references on reactive path planning


## Path smoothing

Ideally this is done when planning the path


## Path following

When to try to return to the path and when to


## SLAM

Need an online slam that can easily