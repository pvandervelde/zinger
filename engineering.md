# Engineering

The engineering document describes the choices made for the different parts of the rover.

## General

* What needs to be repaired / replaced
* how will it fail
* How to deal with tolerances
* Rover inertia?
* Wheel inertia
* Steering power
* Braking power
* Pick-up of cargo
  * From sides without falling over
* FMEA -> Failure Mode Effects Cricicallity Analysis -> Try to predict failures before they happen
* stability
  * Stability while lifting / lowering
* Brakes + hold power for on the hill / when loading
* System redundancy
* Monitoring
* Wheel slip detection
* CAD -> Should inform:
  * Electrical
  * Centre of Gravity / Center of intertia etc
  * Structural
  * ROS / Gazebo
* Inspiration
  * SPMT - Self-propelled modular transporter: https://www.mammoet.com/equipment/transport/self-propelled-modular-transporter/spmt/

## Design

* Dimensions: The minimal dimensions for the cargo are **0.60m * 0.40m * 0.30m (length * width * height)**.
  If the rover is that size or bigger then the rover dimensions control the bounding box (except for
  height) for path finding and collision calculations. Additionally having a rover that is slightly
  bigger than the cargo will make it easier to place the cargo on the rover. Thus the rover will have
  the following minimum dimensions.
  * Length: minimal 0.60 m -> 0.60m / 0.65m
  * Width: minimal 0.40m -> 0.40m / 0.45m
  * Height: Depends on wheel size + structure
* The minimum rover speed is **2.0 m/s** while carrying cargo over level terrain. In order to reduce
  the potential harm in collisions and ensure enough reaction / braking time it is sensible to limit
  the maximum speed and acceleration.
  * Maximum velocity: **2.5 m/s**
  * Maximum acceleration: **1.0 m/s**

## Safety

* Bumpers on all sides to reduce damage in case of hitting anything.
* Sensors for detecting obstacles. When approaching an obstacle, either turn away from it or
  slow down and come to a stop near the obstacle

## Parts

* Drive modules

## Drive module

* taper lock bushing
* Suspension for individual wheels. At 4.0 m/s hitting anything will be nasty because the swing arms are large with
  heavy weights at the end (the wheels + motors)
  * Telescopic drive shafts exist. We might be able to create something like that ourselves
    * Could have a rectangular shaft inside another rectangular shaft with sliders. As long as there
      are enough supports the torque should be transferred without issue. Also we don't necessarily care
      a lot about the amount of sliding friction because the wheel assembly has some weight and we're not
      expecting to have to change the ride height extremely quickly
      * Note that this approach could bind if the height changes while there's torque on the system
        (because torque causes friction etc.)


### Electronics for motors

* Motor drives
* Speed sensors
* Shunt regulator - Handles overvoltage if a motor steps down etc.
* Slew rate generator - how fast / slow can you change the motor speed -> Because gearboxes don't like quick changes


### Steering system - Mechanics

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
  * Use belts for the drive system and gears for the steering
  * Should really have a cover over the bottom so that we don't get dirt in the drive system

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
* SLAM / Mapping
  * https://medium.com/robotics-weekends/2d-mapping-using-google-cartographer-and-rplidar-with-raspberry-pi-a94ce11e44c5
* Misc
  * https://team900.org/labs/
* Path generation
  * smooth path generation using splines
  * Ideally have limit outsides for obstacles
  * Make sure that the outside motors are the limit on speeds
  * At waypoints we don't need to stop just keep direction, speed and acceleration
    the same on both sides of the way point
  * For turns we need to calculate the maximum speeds, both for wheel maximum speed
    and also for wheel direction reversals and cargo / rover minimum radius / tip over etc.
  * Trajectory planning - Path planning with a time component that describes the different velocities
* Communication
  * Push
  * Pull
  * Pub / Sub
  * Pub to blackboard (only keep last value)
* Logs
* Telemetry
* Safe guards
  * Will need a shutdown
  * Load guards
  * Roll-over etc.
  * Human detection / Damage detection
  * Software security
    * Encryption
    * Trust roots
    * Access permissions
    * Audits
* Architecture
  * Layers
    * Hardware interaction
    * Processing
    * Goal level
  * Lower layers run constantly + interrupts / blocks from high level (see: https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.110.809&rep=rep1&type=pdf)
    * decision tree approach
  * Middleware / Comms
* Configuration management -> Pushing new software versions, new commands etc.
  * Default way of pushing changes
  * Audit log
* Links
  * https://robops.org/manifesto
* Sensors
  * Leaky integrators everywhere --> slowly acquire fault states

## Electrical system - Power and data

* Distributed power architecture
  * https://www.digikey.co.nz/en/articles/why-and-how-to-use-a-component-based-distributed-power-architecture-for-robotics
  * https://newsite.ctr-electronics.com/power-distribution-panel/
* Wiring: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/robot-wiring-guide.pdf
* Battery
* Wiring for data
  * Either EtherCAT or CAN. CAN seems to be the most widely supported and has great resilience against disturbances
    * See: https://www.botblox.io/blogs/learn/what-s-the-best-communication-bus-for-robots-and-drones and others
* Power and data busses: Easy decoupling / replacing
* Safety switches
  * Global
  * Local
  * Depower sections / whole rover
  * Depower motor circuits
  * Depower logic circuits

## Electronics - board

* Probably need a dedicated board for orchestrating the different modules because they will need to
  be synchronised. Need some kind of real time control and fault detection
  * Each module has at least 1 motor controller (for 2 motors) and some kind of controller board
    that tells the motor controller(s) what to do
  * Need to feedback current rotation rates and positions back from the module to the overall
    system
  * Would be good to feedback suspension state as well.

## Electronics - sensors

* Laser ground sensor (like a laser mouse) for ground tracking and speed / rotation
* Combine multiple sensors for travel speeds and rotations
* Link a LED to each sensor and blink it when the sensor spots something. This is for debugging
* For IR detectors we may have to change the sensitivity, e.g. when we are in a corner we'll see
  a lot of detections, this will overload our processing. In the open field we can
* When reading rotation speeds for the wheels you need to read off the wheels if you have a diff,
  because otherwise you don't know how much the wheels have moved
  * When reading off the wheel note that for slow movement this is probably in accurate unless we
    have lots of sensor 'points' for a single rotation (but then we might not know where we came from?)
* Temperature sensors for the motors

### Electronics - Light Colors

* Red - Power, rear light, brake light
* Green - Data transfer
* Blue -
* Orange - Motor activated, direction indicators
* Yellow - Sensor activated
* White - head light

Note that the head light, the rear light and the direction indicator are as signal to human operators
not for other robots

## Value Engineering

Improve the ratio between value and cost, either by reducing cost or improving value. Value can be carrying capacity,
speed, battery life etc.

### Navigation

* High level commands for motors are normally a velocity and an angle / direction which are then translated into
  rotational velocity and direction for the motors. However for a swerve drive we have more degrees of freedom so
  the high level commands can specify a velocity vector (where is the rover going) and a 'pointing' vector (where
  is the rover pointing). At a lower level this can be translated into a rotation and a velocity, combined with the
  final pointing direction.
  * From there we need to work out the rotational direction of each wheel, based on where we want to go and the
    speed at which we want to change direction (steer in the same direction vs front and back steering in the
    opposite direction)
* Need trajectory planning - Create a path to the goal, but also deal with the need to smoothly change the wheel
  speed and direction. Additionally for a swerve drive we also need to figure out what direction we need to face in
  * Temporal planning of movement direction, rotations, and the direction the robot is facing
