# Design brief

This document records the specifications for the CrateRover, a mobile robot which carries heavy
or unwieldy loads.


## Purpose

The purpose for CrateRover is to liberate humans from having to carry heavy or unwieldy loads
over semi-flat terrain. The rover will be able to navigate mildly uneven terrain with a load in
a safe manner, without being a danger to humans, animals, property or the cargo.

## Requirements

The following are the must-have requirements for the rover.

1) The CrateRover is designed to cooperate with human operators. As such it will behave in ways
   that prevent any damage to humans, animals, property and the cargo.
   1) The rover will be able to detect the presence of humans and animals. Motion will be stopped
      if there is a possibility of damage.
   1) The rover will possess at least 1 mobility stop (m-stop) which, when engaged, will stop
      all motion immediately. When the mobility stop is reset movement will continue with
      a delay. During the delay the rover will ascertain that no danger exists to humans or animals
      if the motion were to continue.
1) The rover must be able to carry cargo with a maximum weight of **50.0 kg** with a minimum
   bounding box of **0.60m x 0.40m x 0.30m (L x W x H)**. This weight and size allows the rover to
   carry a reasonable size crate with contents.
   1) **Limitation:** Assume that the height of the rover and cargo will be smaller than the minimum
      height of any obstacles. In other words the rover will not need to take heights into account
      while it navigates.
   1) **Limitation:** Assume that the length and width of the cargo will not be greater than the
      length and width of the rover respectively. In other words the rover will not take into
      account the dimensions of the cargo while it navigates.
   1) **Limitation:** The rover will not be able to load or unload the cargo. It is assumed that a
      human operator will handle the safe and secure loading, securing and unloading of the cargo.
1) The rover will be able transport cargo from a given start location to a given destination
   location.
   1) The rover will be able to navigate from its current position to the current
      location of the cargo. Path finding may be done either in advance or while moving.
   1) The rover will be able to pick up cargo from locations with a minimum usable area equivalent
      to 150% of the size of the rover and the cargo combined.
   1) Once the cargo is loaded and secured the rover will be able to navigate to the destination
      location. Path finding may be done either in advance or while moving.
   1) The rover will be able to navigate tight turns
   1) Upon arrival at the destination the rover will be able to position itself in an orientation
      that simplifies the unloading.
   1) **Limitation:** Assume that the maximum velocity on flat ground will be no more than
      **2.0 m/s**. The velocity on sloped ground may be reduced.
   1) **Limitation:** Assume that the maximum slope angle for transports with the maximum weight
      will be no more than **10 degrees**.
   1) **Limitation:** The rover will not have a suspension system. Due to the low speed it is assumed
      that suspension is not required.
1) The rover will be able to communicate status and progress.

1) The rover will be easy to construct and maintain


## Project risks

The CrateRover project has a number of areas that pose a high risk of failure. Factors that contribute
to the risk profile are:

* Complexity
* Unfamiliarity with the area

In order to increase the changes of making the CrateRover project successful it is important to
investigate the areas that form the greatest risk for the project first. The following are the
areas of risk, starting with the highest risk area.

* **Design and construction of the steering & suspension system** - The steering and suspension
  system is one of the key parts of the CrateRover. Additionally it is probably the most complicated
  part. The risk factors for the steering and suspension system are:
  * Attaching the assembly to the frame such that it allows both propulsion and steering.
  * Construction of the assembly such that it is easy to construct and maintain while being
    assembled with sufficient accuracy.
  * Software control of the assembly.
* **Selection of the drive and steering motors** - The steering and drive motors need to be appropriately
  sized for the loads they are required to handle. The risk factors for the selection of the
  drive and steering motors are:
  * The sizing of the appropriate motor and gearing to deliver the required velocities and torque.
  * The selection of the electronic components to control the motors.
* **Electrical system** - The sizing, construction and safety of the electrical system are important
  to ensure that the CrateRover can safely perform to the required demands. The risk factors for the
  electrical system are:
  * The safety of the electrical system.
  * The sizing and construction of the power system, including charge, storage and discharge
    operations.
  * The placement of electrical lines such that they are safe to work with and reduce interference
    with other cables to a minimum.
* **Software** - The software that controls the rover functions needs to be able to deal
  with both the high level functions, like navigating the rover to the goal, as well as the low
  level functions, like controlling the motors and avoiding obstacles. The highest risk factors for
  the software are:
  * The layering and architecture of the software, such that both lower level and higher level
    functions can communicate when necessary without adding unnecessary complexity.
  * Testing changes to the software in a safe and consistent manner.
  * Software performance to ensure rapid responses to changes.
  * Path finding and goal achieving algorithms which allow the rover to transport its cargo to the
    destination in a safe and efficient way.
* **Design of the chassis** - The chassis holds all the parts together while distributing the loads
  from the cargo and motion. The risk factors for the chassis are:
  * Sizing of the chassis parts such that they are able to handle the loads put on them without
    excessive deformation.
  * Assembly of the chassis and attaching the other components such that they are easy to
    assemble and maintain.

## Design decisions

* Dimensions: The minimal dimensions for the cargo are 0.60m * 0.40m * 0.30m (length * width * height).
  If the rover is that size or bigger then the rover dimensions control the bounding box (except for
  height) for path finding and collision calculations. Additionally having a rover that is slightly
  bigger than the cargo will make it easier to place the cargo on the rover. Thus the rover will have
  the following minimum dimensions.
  * Length: minimal 0.60 m -> 0.70 / 0.75
  * Width: minimal 0.40m -> 0.50
  * Height: Depends on wheel size + structure
* The minimum rover speed is **2.0 m/s** while carrying cargo over level terrain. In order to reduce
  the potential harm in collisions and ensure enough reaction / braking time it is sensible to limit
  the maximum speed and acceleration.
  * Maximum velocity: **2.5 m/s**
  * Maximum acceleration: **1.0 m/s**
* Bumpers on all sides to reduce damage in case of hitting anything.

### Rover structure

* Structure

### Motion

* Drive and steering
  * Requirements
    * Need to be able to turn tightly
    * Traverse slopes in all directions
  * Possibilities
    * Differential drive with at least two wheels
    * drive wheels separate from steering wheels
    * drive the steering wheels
    * drive all wheels, steer minimum wheels
    * drive all the wheels, steer all the wheels (swerve drive)
  * The rover will be steered and driven using a swerve drive, i.e. a drive system in which all
    wheels are powered and all wheels are able to rotate 360 degrees infinitely. This provides the
    maximum traction and controllability. The swerve drive provides the ability for the rover to
    move in all directions.

* Wheels
  * Requirements
    * Want to be statically stable, so that there is less chance of falling over and we don't need
      to spend battery power on stability
    * Have to carry a load over uneven terrain (dirt, gravel, concrete, pavers etc.) with bumps,
      dips and small obstacles.
    * Keep the drive system as simple as possible and as cheap as possible -> minimize the number
      of wheels
    * Need to be able to traverse uneven terrain and climb slopes in all directions.
  * Possibilities
    * Wheel numbers: 3, 4, 6, 8. Any less is unstable, any more is more complicated and more expensive.
    * Caster wheels for non-driven wheels
  * Will be using 4 identical wheels which minimizes the complications and cost. Additionally using
    four wheels provides a larger stability envelope.
    * Diameter: **0.20 m**.
    * Width: minimum **0.10 m**
* Suspension - The first version of the rover will not have a suspension.
* Power
  * All electric

### Electronics

* Sensors to detect presence of humans / animals

* communication
  * Status lights on the rover will indicate the current state of the rover.
    * *Green light - solid:* Available for work.
    * *Orange light - solid:* Working on a task.
    * *Orange light - blinking:* Configuring or processing task information.
    * *Red light - blinking:* - Error state. Human intervention needed.

## Roadmap

* Steering system MVPs
  * Simple steering + drive for single unit
  * Software to control single unit
  * Test program to verify accuracy of system
* Electrical system
  * learn electronics
  * Power & data
  * Battery
* Software
  * ROS + Simulation
