# Roadmap

* Motor selection
  * Brushless (see: https://motors.vex.com/brushed-brushless)
    * Smaller
    * Less wear because there are no brushes
    * More expensive
    * More efficient
  * Using a Hall sensor for motor control, or better a Hall sensor for low speed and sensorless for high speed
    * Good at torque at low speed, and handling high dynamic loads
  * If possible find an 'Outrunner' BLDC. This is where the outside of the motor turns. It's better for torque / low speed
  * Power distribution done by Field oriented control
  * Heat control? We need the motors to be able to run over a long period of time without it overheating
    * Heatsink
    * Cooling
  * Power / Current control -> don't want to draw too much power

  * Drive motor -> Power + speed + controller. Ideally fewer parts is better
  * Steering motor -> Power + accuracy. Ideally fewer parts is better
    * DC motor
    * Full rotation servo
    * Stepper motor

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
    * https://scholar.google.co.nz/scholar?start=70&q=software+architecture+for+autonomous+mobile+robot&hl=en&as_sdt=0,5&as_vis=1
    * http://eprints.utm.my/id/eprint/18635/1/DayangNorhayatiJawawiPFSKSM2010.pdf
    * https://arxiv.org/pdf/1811.03563.pdf
    * http://www.cs.ait.ac.th/~mdailey/papers/Limsoonthrakul-Arch.pdf
    * http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/rob2-04-robot-architectures.pdf
    * https://swerveroboticsystems.github.io/DDR/Software/Software.pdf
  * ROS + Simulation
  * Fault tolerance:
    * https://hal-lirmm.ccsd.cnrs.fr/lirmm-01241181/document
    * https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.218.9945&rep=rep1&type=pdf
  * Safety: http://www.es.mdh.se/pdf_publications/5663.pdf
  * ROS:
    * https://github.com/ros-controls/ros_controllers/pull/441
    * https://drive.google.com/file/d/1dXeNoHY7kYR1mJWzMM5BktA5nf9RgFMu/view
  * Testing
    * https://www.chiefdelphi.com/t/best-way-to-test-2910-module-swerve-drive-code/359248
  * Misc
    * https://team900.org/labs/

* Electrical system - Power and data
  * Distributed power architecture: https://www.digikey.co.nz/en/articles/why-and-how-to-use-a-component-based-distributed-power-architecture-for-robotics
  * Wiring: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/robot-wiring-guide.pdf
  * Battery

* Steering system MVP - All units steering + frame to attach to
  * Build 4 steering units and connect them via a simple frame
  * Update the software to control all 4 units
  * Update the test program to verify the accuracy of the system

* Electronics - board

* Electronics - sensors

