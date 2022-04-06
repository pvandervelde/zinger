# Roadmap

In order to address the [project risks](README.md#project-risks) the roadmap is laid out such that
the areas with the highest risk are covered as early as possible in the design and implementation
process. The following is the current suggested design and implementation order that should reduce
the project risks.

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

* Software
  * Mapping
  * Path calculations
  * Path following