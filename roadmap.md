# Roadmap

In order to address the [project risks](README.md#project*risks) the roadmap is laid out such that
the areas with the highest risk are covered as early as possible in the design and implementation
process. The following is the current suggested design and implementation order that should reduce
the project risks.


### Things we need

Translating from high level requirements to low level ones:

High level

* Task and behaviour planning
  * The robot should be able to determine it's actions and motions to achieve a given task. The task
    should be able to be provided at high level, e.g. take part x to person Y / location Y, without
    specifying coordinates or something.
* Semantic understanding and reasoning

Medium level

* Motion planning
* Navigation
* Natural Language processing

Low level

* Controls (velocity, position etc.)
* Perception (what is going on around the robot, tracking of objects, etc.)
* Speech

## plan



--> WHAT IS THE EASIEST PROTOTYPE YOU CAN MAKE SO THAT YOU CAN SEE SOMETHING?

--> MAYBE USE ROS AND SIMULATION? **> for the software / sensors / goal reaching etc.

Can use SCUTTLE for a lot of the software testing etc.



* Create a bench with a single cheap motor * Test driving via software
  * Simple steering for a single unit
  * Software to control angle of the unit
  * Software to control the calibration of the unit
  * Test bench to test unit rotation
  * Mobility stop

* Create a bench with a single cheap motor rotating a structure * Test driving accuracy via software

* Create a bench with two cheap motors rotating and driving a wheel
  * Simple steering + drive for single unit
  * Software to control single unit
  * Test program to verify accuracy of system
  * Test bench to fully test a single unit

* Create a frame with 4 steering units with cheap motors
  * Will need some form of PID controller to ensure that all the units point in the correct directions
    and that direction changes are done at the correct velocity
  * Must coordinate rotation of the units and wheel rotation velocity changes
  * Build 4 steering units and connect them via a simple frame
  * Update the software to control all 4 units
  * Update the test program to verify the accuracy of the system

* Motor selection
  * Type of motors
  * Heat control
  * Power / Current control
  * Drive motor
  * Steering motor

* Electronics for motors
  * Motor drives
  * Speed sensors

* Steering system
  * Co*axial rotations

* Steering system MVP * Single unit steering, no drive
  * Simple steering for a single unit
  * Software to control angle of the unit
  * Software to control the calibration of the unit
  * Test bench to test unit rotation
  * Mobility stop

* Steering system MVP * Single unit steering and drive
  * Simple steering + drive for single unit
  * Software to control single unit
  * Test program to verify accuracy of system
  * Test bench to fully test a single unit

* Software
  * Architecture
  * Fault tolerance
  * Safety
  * ROS:
  * Testing

* Electrical system * Power and data
  * power architecture
  * Wiring
  * Battery

* Steering system MVP * All units steering + frame to attach to
  * Build 4 steering units and connect them via a simple frame
  * Update the software to control all 4 units
  * Update the test program to verify the accuracy of the system

* Electronics * board

* Electronics * sensors

* Software
  * Mapping
  * Path calculations
  * Path following