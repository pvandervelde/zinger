# CrateRover

This document records the specifications for the CrateRover, an autonomous mobile robot which
carries heavy or unwieldy loads.

## Purpose

The purpose for CrateRover is to assist humans by carring heavy or unwieldy loads
over uneven terrain. The rover will be able to navigate uneven terrain with a load in
a safe manner, without being a danger to humans, animals, property or the cargo.

## Requirements

### Must have requirements

The following are the must-have requirements for the rover.

1) The CrateRover is designed to cooperate with human operators. As such it will behave in ways
   that prevent any damage to humans, animals, property and the cargo.
1) The rover will be able to receive and action requests to take given cargo from one location
   to another. The request may come via a number of channels, some of those using natural language.
1) The rover must be able to carry cargo with a maximum weight of **50.0 kg** with a minimum
   bounding box of **0.60m x 0.40m x 0.30m (L x W x H)**. This weight and size allows the rover to
   carry a reasonable size crate with contents.
   1) **Limitation:** The rover will not be able to load or unload the cargo. It is assumed that a
      human operator will handle the safe and secure loading, securing and unloading of the cargo.
1) The rover will be able transport cargo from a given start location to a given destination
   location using a self determined trajectory that takes into account the safety of any humans,
   animals and the cargo.
   1) **Limitation:** The rover will be able to pick up cargo from locations with a minimum usable
      area equivalent to 100% of the size of the rover and the cargo combined.
   1) **Limitation:** The rover will be able to navigate tight turns
   1) **Limitation:** Assume that the maximum velocity on sloped ground will be no more than
      **2.0 m/s**. The velocity on flat ground may be higher.
   1) **Limitation:** Assume that the maximum slope angle for transports with the maximum weight
      will be no more than **15 degrees**.
   1) **Limitation:** The rover will be consider having reached its destination if it is no more than
      0.05 meters away from it.
1) The rover will be able to communicate status and progress.
1) The rover will be easy to construct and maintain

### Nice to have requirements

1) The rover will be able to carry an oversize load safely to the destination.
1) The rover will be able to request assistance from a human operator if required. The request for
   help will be done in natural language and specific. Such that the human operator can quickly
   determine what needs to be done.
1) The rover will be able to cooperate with other rovers to carry large and oversize loads to the
   destination location. The rovers will collaborate to move the cargo safely and keeping in mind
   the stability limits for the cargo. Any issues with cargo stability and safety will be reported
   to human operators before and during the journey.

## Project risks

The CrateRover project has a number of areas that pose a high risk of failure. Factors that contribute
to the risk profile are:

* Complexity
* Unfamiliarity with the area

In order to increase the changes of making the CrateRover project successful it is important to
investigate the areas that form the greatest risk for the project first. The following are the
areas of risk, starting with the highest risk area.

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
  * Swerve control
* **Design and construction of the steering & suspension system** - The steering and suspension
  system is one of the key parts of the CrateRover. Additionally it is probably the most complicated
  part. The risk factors for the steering and suspension system are:
  * Attaching the assembly to the frame such that it allows both propulsion and steering.
  * Construction of the assembly such that it is easy to construct and maintain while being
    assembled with sufficient accuracy.
  * Software control of the assembly.
* **Design of the chassis** - The chassis holds all the parts together while distributing the loads
  from the cargo and motion. The risk factors for the chassis are:
  * Sizing of the chassis parts such that they are able to handle the loads put on them without
    excessive deformation.
  * Assembly of the chassis and attaching the other components such that they are easy to
    assemble and maintain.
* **Selection of the drive and steering motors** - The steering and drive motors need to be appropriately
  sized for the loads they are required to handle. The risk factors for the selection of the
  drive and steering motors are:
  * The sizing of the appropriate motor and gearing to deliver the required velocities and torque.
  * The selection of the electronic components to control the motors.

## Design decisions

### Rover structure

* The rover structure will consist of aluminium extrusions as they are easy to obtain, cut to length
  and connect.

### Motion

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
    * Wheel numbers: 4, 6, 8. Any less is unstable, any more is more complicated and more expensive.
    * Caster wheels for non-driven wheels
  * Will be using 4 identical wheels which minimizes the complications and cost. Additionally using
    four wheels provides a larger stability envelope.
    * Diameter: **0.20 m** based on the idea that we want the rover to be able to traverse obstacles
      of 10cm height, i.e. half the tire diameter.
    * Width: minimum **0.10 m** based on the idea that we want enough surface area to distribute the
      total weight of rover and cargo over a big enough area to reduce ground pressure.
* Drive and steering
  * Requirements
    * Want to be able to turn around the rover center so that we take tight turns and reverse
      in place.
    * Want to be able to traverse in all directions for precision positioning
    * Need to be able to traverse slopes in all directions
  * Possibilities
    * Differential drive with at least two wheels
    * drive wheels separate from steering wheels
    * drive the steering wheels
    * drive all wheels, steer minimum wheels
    * drive all the wheels, steer all the wheels (swerve drive)
  * Rejected
    * Differential drive either requires casters, or multiple drive wheels per rover side. Using
      casters limits slope traversing due to the fact that the drive wheels need to . Using multiple
      drive wheels per rover side adds friction and increases demands on the drive motors.
    * Steering some of the wheels and driving the other wheels. While technically one of the most
      simple solutions, this option limits steering capability and also provides reduced traction.
  * The rover will be steered and driven using a swerve drive, i.e. a drive system in which all
    wheels are powered and all wheels are able to rotate 360 degrees infinitely. This provides the
    maximum traction and controllability. The swerve drive provides the ability for the rover to
    move in all directions. Finally all the wheel units are the same, thus allowing for a modular
    build.
* Suspension - ????
* Power - The first version of the rover will be all electric
* Motion control - The first version of the rover will be controlled by the user. No autonomous
  drive will be provided.

### Electronics

* Sensors - The first version of the rover will only have sensors used to control the motors.
* communication
  * Status lights on the rover will indicate the current state of the rover.y
    * *Green light - solid:* Available for work.
    * *Orange light - solid:* Working on a task.
    * *Orange light - blinking:* Configuring or processing task information.
    * *Red light - blinking:* - Error state. Human intervention needed.
