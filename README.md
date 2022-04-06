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
1) The rover will be able to perform its tasks in both an indoor and an outdoor environment. Use
   in harsh environments should not impede the rover from performing its tasks, nor should it
   lead to damage.
1) The rover will be able to communicate status and progress.
1) The rover will be easy to construct and maintain.

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

* Complexity in both the underlying field and the design
* Unfamiliarity with the underlying field

In order to increase the changes of making the CrateRover project successful it is important to
investigate the areas that form the greatest risk for the project first. The following are the
areas of risk, starting with the highest risk area.

* **Software** - The software that controls the rover functions needs to be able to deal
  with both the high level functions, like navigating the rover to the goal, as well as the low
  level functions, like controlling the motors and avoiding obstacles. The highest risk factors for
  the software are:
  * The layering and architecture of the software, such that both lower level and higher level
    functions can communicate when necessary without adding unnecessary complexity.
  * Testing changes to the software in a safe and consistent manner.
  * Software performance to ensure rapid responses to changes.
  * Goal achieving algorithms which allow the rover to transport its cargo to the
    destination in a safe and efficient way.
  * Trajectory planning, path following and location
  * Control of the drive and steering system
* **Electrical system** - The sizing, construction and safety of the electrical system are important
  to ensure that the CrateRover can safely perform to the required demands. The risk factors for the
  electrical system are:
  * The safety of the electrical system.
  * The sizing and construction of the power system, including charge, storage and discharge
    operations.
  * The placement of electrical lines such that they are safe to work with and reduce interference
    with other cables to a minimum.
* **Design and construction of the steering & suspension system** - The steering and suspension
  system is one of the key parts of the CrateRover. Additionally it is probably the most complicated
  part. The risk factors for the steering and suspension system are:
  * Attaching the assembly to the frame such that it allows both propulsion and steering.
  * Construction of the assembly such that it is easy to construct and maintain while being
    assembled with sufficient accuracy.
  * Software control of the assembly.
