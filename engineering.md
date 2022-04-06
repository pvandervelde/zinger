# Engineering

The engineering document describes the choices made for the different parts of the rover.

## Translating the requirements

In order to determine the shape, size and exact characteristics of the rover we first need to
translate the requirements into more concrete information.

### Must-Have Requirement: Cooperating with human operators

The high level requirement is:

    The CrateRover is designed to cooperate with human operators. As such it will behave in ways
    that prevent any damage to humans, animals, property and the cargo.

Translating this requirement leads to the following engineering demands:

* The rover needs to have sensors that can detect stationary and moving obstacles
* The rover should be able to predict with some accuracy the path that will be taken by moving
  obstacles and use this information for obstacle avoidance and trajectory planning
* The rover should be able to adapt the trajectory due to changes in directions of obstacles
* The rover will provide safe methods of stopping the rover, either remotely or up close
* The rover will need to be able to deal with failures in a safe manner

### Must-Have Requirement: Task planning

The high level requirement is:

    The rover will be able to receive and action requests to take given cargo from one location
    to another. The request may come via a number of channels, some of those using natural language.

Translating this requirement leads to the following engineering demands:

* The rover needs to be able to receive and process high level tasks, e.g. collect item X from
  location A and take it to location B, where location definitions may be fuzzy, e.g. a description
  rather than an exact coordinate.
* The rover needs to be able to communicate the plan for achieving the provided goal and provide
  progress updates while it executes the task.
* The rover should be able to understand a, potentially limited, form of natural language.

### Must-Have Requirement: Navigation

The high level requirement is:

    The rover will be able transport cargo from a given start location to a given destination
    location using a self determined trajectory that takes into account the safety of any humans,
    animals and the cargo.

This requirement comes with a number of limitations. Specifically:

1) **Limitation:** The rover will be able to pick up cargo from locations with a minimum usable
  area equivalent to 100% of the size of the rover and the cargo combined.
1) **Limitation:** The rover will be able to navigate tight turns
1) **Limitation:** Assume that the maximum velocity on sloped ground will be no more than
  **2.0 m/s**. The velocity on flat ground may be higher.
1) **Limitation:** Assume that the maximum slope angle for transports with the maximum weight
  will be no more than **15 degrees**.
1) **Limitation:** The rover will be consider having reached its destination if it is no more than
  0.05 meters away from it.

Translating this requirement with the limitations leads to the following engineering demands:

* The rover will be able to plan a trajectory, i.e. a path with a velocity and acceleration components,
  from the start position to the goal position.
* The trajectory planning will take into account the stability of the cargo and rover at any point
  on the trajectory.
* Static and dynamic detection of in-appropriate paths, e.g. detecting inappropriate slopes while
  moving and being able to update the planned trajectory with the information.
* The rover will be able to monitor the status of the cargo, specifically for the stability perspective.
* The rover will possess all-wheel steering, with 360 degree rotation. This allows both tight turns
  as well as in place rotations and crab moves, which may all be necessary for accurate placement.
* Trajectory planning needs to take into account the provided limitations on performance as well as
  structural limits and limits on the drive and steering system.
* Construction of the drive and steering system should be such that the location of the rover can
  be achieved to the required accuracy.

### Must-Have Requirement: Carrying cargo

The high level requirement is:

    The rover must be able to carry cargo with a maximum weight of **50.0 kg** with a minimum
    bounding box of **0.60m x 0.40m x 0.30m (L x W x H)**. This weight and size allows the rover to
    carry a reasonable size crate with contents.

This requirement comes with a number of limitations. Specifically:

1) **Limitation:** The rover will not be able to load or unload the cargo. It is assumed that a
  human operator will handle the safe and secure loading, securing and unloading of the cargo.

Translating this requirement with the limitations leads to the following engineering demands:

* The maximum cargo weight is 50 kg
* The rover should at least be able to carry a box of size 0.60m x 0.40m x 0.30m
* The rover will provide a means of securing the cargo

### Must-Have Requirement: Environments

The high level requirement is:

    The rover will be able to perform its tasks in both an indoor and an outdoor environment. Use
    in harsh environments should not impede the rover from performing its tasks, nor should it
    lead to damage.

Translating this requirement leads to the following engineering demands:

* The rover will be able to traverse different surface types. The minimum expectation is
  * Concrete
  * Asphalt
  * Hard packed dirt
  * Sand
  * Light mud
* The rover is shielded as much as possible from ingress of foreign materials into the sensitive
  components of the rover.
* The rover will provide easy ways for foreign materials to be removed from the rover, either
  automatically or with the assistance of an operator.
* It should be possible to clean the rover without damaging it or the operator doing the cleaning.

### Must-Have Requirement: Communication

The high level requirement is:

    The rover will be able to communicate status and progress.

Translating this requirement leads to the following engineering demands:

* The rover needs one or more ways in which it can communicate its status, both for local
  and remote operators.
* The rover needs one or more ways in which it can communicate the progress of the current task.

### Must-Have Requirement: Construction

The high level requirement is:

    The rover will be easy to construct and maintain

Translating this requirement leads to the following engineering demands:

* The rover should be made from standard materials and parts as much as possible
* The rover should be able to be assembled with simple tools
* The assembly order should be obvious
* It should be possible to easily remove parts for replacement or repair
* It should be easy to maintain those parts that need maintenance

### Nice-To-Have Requirement: Oversize loads

The high level requirement is:

    The rover will be able to carry an oversize load safely to the destination.

Translating this requirement leads to the following engineering demands:

* The rover should be able to determine the shape and size of the cargo.
* The rover should be able to determine the position of the cargo relative to its own chassis.
* Trajectory planning should take into account the larger foot print of the rover with cargo, including
  turning radius and slope start and end.
* The rover will provide means of securing oversized cargo.

### Nice-To-Have Requirement: Request assistance

The high level requirement is:

    The rover will be able to request assistance from a human operator if required. The request for
    help will be done in natural language and specific. Such that the human operator can quickly
    determine what needs to be done.

Translating this requirement leads to the following engineering demands:

* The rover will provide means of communication, via text or voice
* The rover will be able to determine the cause of any issues to such a degree that it can provide
  a clear description of the issue to a human operator
* The rover will be able to provide specific instructions to resolve the issue to a human operator.
  This includes providing the operator with the appropriate context

### Nice-To-Have Requirement: Multi-rover tasks

The high level requirement is:

   The rover will be able to cooperate with other rovers to carry large and oversize loads to the
   destination location. The rovers will collaborate to move the cargo safely and keeping in mind
   the stability limits for the cargo. Any issues with cargo stability and safety will be reported
   to human operators before and during the journey.

Translating this requirement leads to the following engineering demands:

* The rover will be able to communicate with other rovers.
* The rover will be able to determine which other rovers are able to form a team.
* The rovers will be able to create a plan. This plan will include the tasks for each rover.
* While performing the task the rovers will coordinate amongst themselves.

## Component requirements

Using the engineering demands we can come up with the detailed requirements for the components that
make up the rover. For the purposes of this document we split discussion into three parts, the overall
dimensions and performance, the requirements for the hardware and finally the requirements for the
software.

The following tree provides an idea of what will be discussed in the following sections.

* Dimensions and performance
* Safety
* Hardware
  * Chassis
  * Drive system
  * Steering system
  * Electronics
  * Electrical
  * Sensors
* Software
  * Architecture
    * Capabilities
    * Decision tree vs state machine
    * Fault analysis / handling
    * Which ROS nodes --> Capabilities
    * Connectivity between nodes
    * Data flow
    * Data processing
    * Task planning
  * Algorithms
    * SLAM
    * Path planning
    * Vision
    * Language processing
  * Controllers
    * Steering and drive --> Probably a single controller because steering and drive are strongly
      connected in swerve
    * Sensors
    * Recovery
    * Fault handling

### Dimensions and performance

The general dimensions and performance requirements for the rover are as follows:

* Dimensions: The minimal dimensions for the cargo are **0.60m * 0.40m * 0.30m (length * width * height)**.
  The minimum dimensions for the rover will be slightly larger than the minimum cargo dimensions.
  Stability, structural and performance requirements may increase one or more dimensions of the rover.
  Thus the rover will have the following minimum dimensions.
  * Length: minimal 0.65m
  * Width: minimal 0.45m
  * Height: Depends on wheel size + structure
* The maximum rover speed is set to be fast enough to move cargo at a reasonable rate while providing
  the rover with enough time to react when obstacles appear. Based on this the rover should be able
  to move at **2.0 m/s** in a straight line while carrying cargo over level terrain. Further limitations
  are set to other speeds and accelerations. The deceleration requirements are set such that the rover
  can come to a full stop from its maximum velocity in less than 1 meter of distance.
  * Maximum linear velocity without cargo: **2.0 m/s**
  * Maximum linear velocity with cargo: **2.0 m/s**
  * Maximum linear acceleration: **1.0 m/s^2**
  * Minimum linear deceleration: **4.0 m/s^2**
  * Minimum linear velocity: **0.10 m/s**
* The obstacle avoidance update rate (Hz): **TBD**
* Response times
  * Object detection: **TBD**
  * Obstacle response: **TBD**

### Safety

As the rover moves around in a dynamic environment it is paramount that it is able to do so in a safe
manner. There are many different areas that need to be considered in order to design and build a rover
that can navigate the world in a safe manner. For the time being these areas and their considerations
are as follows.

* **Task planning:** Ensure that tasks can be executed safely. This involves determining if the specific
  cargo can safely be carried, if there is a safe path to transport the cargo from origin to destination,
  what the minimum and maximum acceleration and deceleration limits are etc.. This information can then
  be used in other safety related areas to determine their limits. For instance the combined dimensions
  of rover and cargo may be used by the trajectory planning algorithms to determine a safe path.
* **Trajectory planning:** To plan a safe trajectory from the origin to the destination, taking into
  account the conditions of the surroundings and the state of the cargo, e.g. the maximum slope angle,
  minimum passage size, safe turning radii, safe velocities in areas with traffic etc..
* **Obstacle detection:** Ensure that all obstacles can be detected in varying conditions


* Local path planning: avoid obstacles
* Status: Being able to detect / prevent unsafe positions / states
  * Connectivity between electrical systems
  * Connectivity between data systems
  * Errors in sensors --> No signal isn't good (but in some cases it means no detections??)
  * Robot responses to mechanical failure
    * drive system
      * Wheel rotation speeds
      * Belt slippage
      * Wheel vertical and sideways accelerations
      * Optical slip-rings and wireless power could be used to get power to
        the wheel system and data back from sensors around the wheel
  * Cargo stability
  * Errors in detection
  * Sensor disagreements
  * Loss of connection -> Should be handled without issue. If there is a task running then we continue with that task, safely

* Be visible through colour, lights and sounds
  * Ideally the sound won't be a continuous beeping noise because that's annoying

* Failure handling
  * Safe ways to halt / Safe locations to halt
  * Diagnose and try to remediate
    * Do a re-calibration,  zero the encoders / motors
    * Restart bits of the code / hardware
  * Ways to minimize damage in case of failure
    * If the rover is going to crash, attempt to move in a way that minimizes damage to people, animals and the cargo
    * reduce collision / impact loads with bumpers and strapping
      * Design the bumpers so that they absorb enough energy
  * Failure to make goal
    * Depends on the reason
      * Goal isn't reachable --> Inform the humans
      * Failure of some sub-system --> Either stop, or return to base?
      * Battery running low --> Back to charge station + inform human



* FMEA -> Failure Mode Effects Criticality Analysis -> Try to predict failures before they happen


### Hardware

#### General design

* Will be using 4 identical wheels which minimizes the complications and cost. Additionally using
    four wheels provides a larger stability envelope.
  * Diameter: **0.20 m** based on the idea that we want the rover to be able to traverse obstacles
      of 10cm height, i.e. half the tire diameter.
  * Width: minimum **0.05 m** based on the idea that we want enough surface area to distribute the
      total weight of rover and cargo over a big enough area to reduce ground pressure.
* The rover will be steered and driven using a swerve drive, i.e. a drive system in which all
    wheels are powered and all wheels are able to rotate 360 degrees infinitely. This provides the
    maximum traction and controllability. The swerve drive provides the ability for the rover to
    move in all directions. Finally all the wheel units are the same, thus allowing for a modular
    build.
  * Maximum speed at which we can turn (i.e. change direction) of the wheel unit: XXXX
  * Acceleration profiles

#### Chassis



* Pick-up of cargo
  * From sides without falling over
* stability
  * Stability while lifting / lowering
  * Stability while traversing slopes
  * Stability while maneuvering
* How to deal with tolerances
  * Tolerance for fit etc.
  * Tolerance for the final parts, e.g. how much can the frame be out of square without influence on
    the robots behaviour
* The rover structure will consist of aluminium extrusions as they are easy to obtain, cut to length
  and connect.

#### Drive system

* Modular drive system, because we have 4
* Braking power
* Brakes + hold power for on the hill / when loading
* Wheel slip detection
* taper lock bushing
* Suspension for individual wheels. At 2.5 m/s hitting anything will be nasty because the swing arms are large with
  heavy weights at the end (the wheels + motors)
  * Telescopic drive shafts exist. We might be able to create something like that ourselves
    * Could have a hex / rectangular shaft inside another similar shaped shaft with sliders. As long as there
      are enough supports the torque should be transferred without issue. Also we don't necessarily care
      a lot about the amount of sliding friction because the wheel assembly has some weight and we're not
      expecting to have to change the ride height extremely quickly
      * Note that this approach could bind if the height changes while there's torque on the system
        (because torque causes friction etc.)
* Motor drives
* Speed sensors
* Shunt regulator - Handles overvoltage if a motor steps down etc.
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

#### Steering system

* Accuracy
* Rotational step size (less than 1 degree) - sensors
* Keeping steering motors aligned - handling drift

#### Electronics

* System redundancy

* Red - Power, rear light, brake light
* Green - Data transfer
* Blue -
* Orange - Motor activated, direction indicators
* Yellow - Sensor activated
* White - head light

Note that the head light, the rear light and the direction indicator are as signal to human operators
not for other robots

* Probably need a dedicated board for orchestrating the different modules because they will need to
  be synchronised. Need some kind of real time control and fault detection
  * Each module has at least 1 motor controller (for 2 motors) and some kind of controller board
    that tells the motor controller(s) what to do
  * Need to feedback current rotation rates and positions back from the module to the overall
    system
  * Would be good to feedback suspension state as well.
* Fault handling for if one of the steering or drive motors doesn't respond

#### Electrial Power and data

* System redundancy
* Distributed power architecture
  * <https://www.digikey.co.nz/en/articles/why-and-how-to-use-a-component-based-distributed-power-architecture-for-robotics>
  * <https://newsite.ctr-electronics.com/power-distribution-panel/>
* Wiring: <https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/robot-wiring-guide.pdf>
* Battery
* Wiring for data
  * Either EtherCAT or CAN. CAN seems to be the most widely supported and has great resilience against disturbances
    * See: <https://www.botblox.io/blogs/learn/what-s-the-best-communication-bus-for-robots-and-drones> and others
* Power and data busses: Easy decoupling / replacing
* Safety switches
  * Global
  * Local
  * Depower sections / whole rover
  * Depower motor circuits
  * Depower logic circuits
* Battery safety

#### Sensors

* System redundancy
* Monitoring
* Bumpers on all sides to reduce damage in case of hitting anything.
* Sensors for detecting obstacles. When approaching an obstacle, either turn away from it or
  slow down and come to a stop near the obstacle
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

### Software


#### Architecture

* Safety layer
  * Check if there are humans / animals in our path
  * Check if our cargo is safe, and will be safe in our next movements
* Failure handler
  * Predictive system for failures, e.g. low battery, path planning with hills / drop-offs etc.
* Task planning
  * How to get from where we are to where we wanting to be
    * With and without a map
    * With dynamic obstacles
* Layers
  * Hardware interaction
  * Processing
  * Goal level
  * Lower layers run constantly + interrupts / blocks from high level (see: <https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.110.809&rep=rep1&type=pdf>)
    * decision tree approach
  * Middleware / Comms
  * Decision tree

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

* Testing
  * <https://www.chiefdelphi.com/t/best-way-to-test-2910-module-swerve-drive-code/359248>

* Architecture
  * <https://scholar.google.co.nz/scholar?start=70&q=software+architecture+for+autonomous+mobile+robot&hl=en&as_sdt=0,5&as_vis=1>
  * <http://eprints.utm.my/id/eprint/18635/1/DayangNorhayatiJawawiPFSKSM2010.pdf>
  * <https://arxiv.org/pdf/1811.03563.pdf>
  * <http://www.cs.ait.ac.th/~mdailey/papers/Limsoonthrakul-Arch.pdf>
  * <http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/rob2-04-robot-architectures.pdf>
  * <https://swerveroboticsystems.github.io/DDR/Software/Software.pdf>
* Fault tolerance - of software issues:
  * <https://hal-lirmm.ccsd.cnrs.fr/lirmm-01241181/document>
  * <https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.218.9945&rep=rep1&type=pdf>

#### Algorithms

* Slew rate generator - how fast / slow can you change the motor speed -> Because gearboxes don't like quick changes
* Movement and predictability - People are pretty good with predicting smooth movements, but
  bad with jerky movements

* Task planning
  * Cargo dimensions and weight
    * Can it safely be carried
    * Are the dimensions safe for travel
    * What are the limits for slope angle, velocity, acceleration, deceleration, turning radius, swing radius when
      turning around the c.o.g.
  * Origin / destination
    * Can it safely be loaded, e.g. are we on a steep slope
    * Is there a safe path from the origin to the destination
    * Will be iterative with trajectory planning
  * State of charge - Can we reach the destination and have enough battery charge to get to a
    charge station
* Trajectory planning
  * smooth path generation using splines
  * Ideally have limit outsides for obstacles
  * Make sure that the outside motors are the limit on speeds
  * At waypoints we don't need to stop just keep direction, speed and acceleration
    the same on both sides of the way point
  * For turns we need to calculate the maximum speeds, both for wheel maximum speed
    and also for wheel direction reversals and cargo / rover minimum radius / tip over etc.
  * Trajectory planning - Path planning with a time component that describes the different velocities
  * Keep in mind state-of-charge. Potentially don't accept missions that are too long
  * Have multiple path planning algorithms. Some that use a complete map and some that allow unknown
    terrain. Switch between the algorithms.
    * Might be worth having some kind of selection method for selecting the best path / trajectory
      planning system
  * Find some way of getting a rough global map and then use reactive planning to go places
    The global map will allow users to request a location, while the reactive planning allows the
    robot to avoid obstacles
  * Determine what we want from a planning algorithm
    * Work with the limitations of the rover (kinematics etc.), including differential drive and 4W steering
    * Robust - Should be able to deal with unexpected obstacles etc.
    * Reliable - It should consistently get the robot to the goal
    * Smooth - G1 continuous is good, G2 continuous is better. Want smooth movement at all times
    * Adaptable - It should be able to adapt to changing conditions / environment and able to replan if required
    * Multi-goal - Able to deal with multiple goals / waypoints in a single planning session. Need
      to have smooth transitions between the goals and prescribed positions / orientations at way points
    * Able to handle actual robot geometry (not all robots are round)
    * Efficient - It should be able to find the shortest path
* Navigation
  * Need some form of path planning. In general there is a global planner and a local one. The global
    planner determines a route in advance and the local planner tries to follow that route while dealing
    with obstacles, either static or dynamic
  * Swerve has the advantage that we can turn the robot body in any direction while continuing along
    the same path, i.e. the orientation of the robot body is decoupled from the direction of motion
  * For our path planners this means that we can turn the robot body to move past obstacles.
    * This could be especially important for cases where the rover is carrying an oversized load
  * Additionally when dealing with hills the decoupling could mean that we can keep the robot in the most
    stable position (e.g. front facing up hill) while still moving along the slope
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
* SLAM / VSLAM
* Object detection
* What are the performance requirements for the algorithms

#### Controllers

* System redundancy
* Monitoring
* Fault handling
* safety
  * <https://robops.org/manifesto>


* Sensors
  * Leaky integrators everywhere --> slowly acquire fault states --> Useful to determine if the robot is stuck
* Modules
  * SLAM -> simultaneous localization and mapping
  * Path planning / Trajectory planning
  * Navigation
  * Odometry
