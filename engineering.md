# Engineering

The engineering document describes the choices made for the different parts of the rover.

## Motor selection

The motor selection chapter discusses the motors that were chosen to drive the rover.

* Type of motors
* Heat control
* Power / Current control

### Selection

There are two motors to select (initially), the drive motor that propels the rover and the steering
motor which turns the wheel assembly so that the rover can turn.
#### Drive motor

#### Steering motor









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
* gearing
  * CVT is apparently possible
* Motor controller

* Don't attach things to the motor shaft if they put a radial load on the shaft because of fatigue

* Gears
  * If we need 90 degree gears
    * https://www.instructables.com/90-Degree-Adapter-for-VexPro-VersaPlanetary-Gearbo/
    * https://www.vexrobotics.com/217-6293.html
  * Gearboxes
    * https://www.andymark.com/products/cim-sport-options
    * https://www.vexrobotics.com/versaplanetary.html
  * CVT is possible for for the time being we don't care too much about efficiency. That will come later

* Drive motor -> Power + speed + controller. Ideally fewer parts is better
* Steering motor -> Power + accuracy. Ideally fewer parts is better
  * DC motor
  * Full rotation servo
  * Stepper motor

## Electronics for motors

* Motor drives
* Speed sensors
* Shunt regulator - Handles overvoltage if a motor steps down etc.
* Slew rate generator - how fast / slow can you change the motor speed -> Because gearboxes don't like quick changes

## Structure

* 1109 Series goRAIL

## Steering system - Mechanics

* Co-axial rotations
* Drive motor position
  * Motors not on turning section
    * Inertia is higher if the drive motor turns too. Possibly the rotating parts of the motor
      also influence the rotation due to gyroscopic forces
    * Power wires eventually tangled after too many turns
    * Motor at wheel height is more susceptible to dirt
    * Reduces clearance because motors take up space
* Drive motor power transfer
  * Options
    * Belts
      * Easier
      * Lighter
      * Allows non-parallel axles
      * Less efficient
      * Requires pre-tension
      * When changing speed or direction there is a lag
    * Chains
      * Easier
      * Lighter
      * More efficient than belt, but less efficient than gears
      * Does not require pre-tension
      * Axles have to be parallel. Little tolerance
      * When changing speed or direction there is a lag
    * Gears
      * Most efficient (if properly set up)
      * Can transfer large amounts of torque
      * Need to be over short distance
      * Noisy at high speed
      * Need lubrication
      * A herringbone or helical gear would be most efficient but those are expensive
  * Should really have a cover over the bottom so that we don't get dirt in the drive system

## Steering system MVP - Single unit steering, no drive

* Simple steering for a single unit
* Software to control angle of the unit
* Software to control the calibration of the unit
* Test bench to test unit rotation
* Mobility stop

## Steering system MVP - Single unit steering and drive

* Simple steering + drive for single unit
* Software to control single unit
* Test program to verify accuracy of system
* Test bench to fully test a single unit

## Software

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

## Electrical system - Power and data

* Distributed power architecture
  * https://www.digikey.co.nz/en/articles/why-and-how-to-use-a-component-based-distributed-power-architecture-for-robotics
  * https://newsite.ctr-electronics.com/power-distribution-panel/
* Wiring: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/robot-wiring-guide.pdf
* Battery

## Steering system MVP - All units steering + frame to attach to

* Build 4 steering units and connect them via a simple frame
* Update the software to control all 4 units
* Update the test program to verify the accuracy of the system

## Electronics - board

## Electronics - sensors

* Laser ground sensor (like a laser mouse) for ground tracking and speed / rotation


## Software

* Path generation
  * smooth path generation using splines
  * Ideally have limit outsides for obstacles
  * Make sure that the outside motors are the limit on speeds
  * At waypoints we don't need to stop just keep direction, speed and acceleration
    the same on both sides of the way point
  * For turns we need to calculate the maximum speeds, both for wheel maximum speed
    and also for wheel direction reversals and cargo / rover minimum radius / tip over etc.


## Value Engineering

Improve the ratio between value and cost, either by reducing cost or improving value. Value can be carrying capacity,
speed, battery life etc.
