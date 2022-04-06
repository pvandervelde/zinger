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
   bounding box of **0.60m x 0.40m x 0.30m (L x W x H)**. This weight and size allows the rover to carry a reasonable size crate with contents.
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
      that simplifies.
   1) **Limitation:** Assume that the maximum velocity on flat ground will be no more than
      **2.0 m/s**. The velocity on sloped ground may be reduced.
   1) **Limitation:** Assume that the maximum slope angle for transports with the maximum weight
      will be no more than **10 degrees**.
   1) **Limitation:** The rover will not have a suspension system. Due to the low speed it is assumed
      that suspension is not required.
1) The rover will be able to communicate status and progress.
   1) Status lights on the rover will indicate the current state of the rover.
      1) *Green light - solid:* Available for work.
      1) *Orange light - solid:* Working on a task.
      1) *Orange light - blinking:* Configuring or processing task information.
      1) *Red light - blinking:* - Error state. Human intervention needed.
1) The rover will be easy to construct and maintain
