# Roadmap

In order to address the [project risks](README.md#project*risks) the roadmap is laid out such that
the areas with the highest risk are covered as early as possible in the design and implementation
process. The following is the current suggested design approach and implementation order that should
reduce the project risks.

## Goal

To see progress in high risk areas as quickly as possible and constantly

## Plan

* Get experience with ROS / Gazebo / RViz / Navigation - Goal: Understand how things work in general
  * Virtual navigation with SCUTTLE in a simulator
    * Test drive code
    * Test navigation and learn how it works
  * Different test environments for testing trajectory planning and navigation
    * Automation in order to test things
      * Compare velocity / acceleration / position
  * Create ROS noetic RPi image with all the tools
    * Run ROS as daemon
    * Handle logs and metrics appropriately
    * package as ubuntu snap
    * Auto-update of changes
* Get experience with physical robots - Goal: Learn about the differences between simulation and physical.
  Also understand physical robots. Finally keep things interesting
  * SLAM for SCUTTLE
    * Implement a bumper
    * Implement ultrasound sensors
  * Design tests for Odometry
  * Design tests for sensors
    * Bumpers
    * Sonar
  * SLAM using camera's
    * Add a camera to SCUTTLE
  * SLAM using TOF
  * Design tests for sensors
    * Camera
    * ToF
  * Real-life navigation with SCUTTLE using ultrasound and bumpers
* Switch to ROS2 for nav - Goal: learn ROS2 because the ROS2 nav stack is more extensible
  * Different path planning / navigation methods
    * Global planning methods
    * Local planning methods
    * Reactive planning
    * Determine what we want from a planning algorithm
      * Work with the limitations of the rover (kinematics etc.), including differential drive and 4W steering
      * Robust - Should be able to deal with unexpected obstacles etc.
      * Reliable - It should consistently get the robot to the goal
      * Smooth - G1 continuous is good, G2 continuous is better. Want smooth movement at all times
      * Adaptable - It should be able to adapt to changing conditions / environment and able to replan if required
      * Multi-goal - Able to deal with multiple goals / waypoints in a single planning session. Need
        to have smooth transitions between the goals and prescribed positions / orientations at way points
      * Multiple sensor inputs
      * Able to handle actual robot geometry (not all robots are round)
      * Able to handle terrain topography - Slopes etc.
      * Able to find a safe path
        * Minimal space requirements
        * Terrain angles
      * Fast - Should be able to decide on a path quickly
      * Efficient - It should be able to find the shortest path
* Get an arm working
* Task planning - Goal: Get a robot to do something useful
  * Trajectory planning with moving obstacles
    * Initially predictable path, both straight and curved
    * Then introduce random movements
  * Task planning
    * Request processing
  * Goal behaviour tested with SCUTTLE
  * Communication of SCUTTLE with humans
    * Lights for indicating driving direction and modes
    * Some kind of app to communicate commands to the rover
    * The ability of the rover to send status updates back
    * Allow the rover to ask questions etc.
* ROS2 with 4 wheel steering - Goal: Get a swerve drive running
  * Simple swerve model in simulator, manual steering
    * Initially use 2 wheel steering
    * Switch to 4 wheel steering, but only opposing
    * Allow 4 wheel steering + crab
  * Swerve navigation
  * Swerve trajectory planning
* Simple physical 4 wheel steering robot - Goal: To move to the physical world and see if it works
  * use a simple model
  * Test out in real life
* Design and build the actual swerve module - Goal: to get our serve module running
  * Create simple single wheel swerve in code and drive it, learn to connect motors etc.
  * Create a bench with a single cheap motor
    * Test driving via software
    * Simple steering for a single unit
    * Software to control angle of the unit
    * Software to control the calibration of the unit
    * Test bench to test unit rotation
    * Mobility stop
  * Create a bench with a single cheap motor rotating a structure
    * Test driving accuracy via software
  * Create a bench with two cheap motors rotating and driving a wheel
    * Simple steering + drive for single unit
    * Software to control single unit
    * Test program to verify accuracy of system
    * Test bench to fully test a single unit
* Build a frame with 4 swerve modules - Goal: To create our own swerve module
  * Create a frame with 4 steering units with cheap motors
    * Will need some form of PID controller to ensure that all the units point in the correct directions
      and that direction changes are done at the correct velocity
    * Must coordinate rotation of the units and wheel rotation velocity changes
    * Build 4 steering units and connect them via a simple frame
    * Update the software to control all 4 units
    * Update the test program to verify the accuracy of the system
    * Build 4 steering units and connect them via a simple frame
    * Update the software to control all 4 units
    * Update the test program to verify the accuracy of the system
