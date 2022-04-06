# Roadmap

In order to address the [project risks](README.md#project*risks) the roadmap is laid out such that
the areas with the highest risk are covered as early as possible in the design and implementation
process. The following is the current suggested design approach and implementation order that should
reduce the project risks.

## Goal

To see progress as quickly as possible and constantly

## Plan


--> WHAT IS THE EASIEST PROTOTYPE YOU CAN MAKE SO THAT YOU CAN SEE SOMETHING?

--> MAYBE USE ROS AND SIMULATION? **> for the software / sensors / goal reaching etc.

Can use SCUTTLE for a lot of the software testing etc.

* Virtual robot driving to learn how to move a robot
  * Turtlebot
  * Virtual SCUTTLE

* Virtual navigation with SCUTTLE in a simulator
  * Test drive code
  * Test navigation and learn how it works

* Different test environments for testing trajectory planning and navigation

* Implement a bumper and ultrasound sensors to SCUTTLE

* Real-life navigation with SCUTTLE using ultrasound and bumpers

* Goal behaviour tested with SCUTTLE

* Create simple single wheel swerve in code and drive it, learn to connect motors etc.

* Simple swerve model in simulator, manual steering
  * Initially use 2 wheel steering
  * Switch to 4 wheel steering, but only oposing
  * Allow 4 wheel steering + crab

* Add uncertainty to the model

* Trajectory planning with moving obstacles
  * Initially predictable path, both straight and curved
  * Then introduce random movements

* Swerve navigation

* Swerve trajectory planning

* Create a bench with a single cheap motor * Test driving via software
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
  * Co-axial rotations

* Steering system MVP - Single unit steering, no drive
  * Simple steering for a single unit
  * Software to control angle of the unit
  * Software to control the calibration of the unit
  * Test bench to test unit rotation
  * Mobility stop

* Steering system MVP - Single unit steering and drive
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

* Electrical system - Power and data
  * power architecture
  * Wiring
  * Battery

* Steering system MVP - All units steering + frame to attach to
  * Build 4 steering units and connect them via a simple frame
  * Update the software to control all 4 units
  * Update the test program to verify the accuracy of the system

* Electronics - board

* Electronics - sensors
