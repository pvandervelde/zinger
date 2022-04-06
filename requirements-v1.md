# Requirements

This document records the specifications for the CrateRover, a mobile robot which carries heavy
or unwieldy loads.


## Purpose

The purpose for CrateRover is to liberate humans from having to carry heavy or unwieldy loads
over semi-flat terrain.

## Must haves

The following are the requirements for the rover that
The rover



* Must have
  * Carry a load
    * Maximum weight: 25.0 kg
    * Size (l * w * h): 0.520m x 0.350m x 0.268m
    * Length no greater than the length of the rover
    * Width no greater than the width of the rover
  * Navigate terrain from current position to the pick-up location
  * Navigate terrain from the pick-up point to the delivery point
    * Taking into account the size and shape of the load
    * Communicate and coordinate between all the rovers involved
  * Communicate status and progress
  * Ability to navigate tight corners
  * Ability to deliver load to an exact location, even with tight entry constraints
  * Ability to work for a 8 hour period
  * Safely work with people
    * Detect presence of humans and stop if movement is dangerous
    * Ability to stop (m-stop - mobility stop) / pause the rover
      * Only kill motion, but not compute & sensors
      * When resetting the switch -> What is the state. Movement should be confirmed somehow
      * Apply brakes
      * Send signal to other robots that are in the same cargo transport
      * Wireless m-stop -> On a separate channel with constant heartbeats.
      * https://www.fortrobotics.com/wireless-emergency-stop
    * Indicate rover status clearly
      * Lights to indicate status
      * Free - GREEN - SOLID
      * Busy - ORANGE - SOLID
      * Configuring - ORANGE - BLINK
      * Error - RED - BLINK
    * Report rover stats / metrics
    * Detection of load
      * Total load weight
      * Differential load weight on the corners of the rover
      * Stability indication
  * Easy to build
    * T-slot aluminium (https://www.tslots.com/wp-content/uploads/2019/11/TSLOTS-Catalog-Website_.pdf)
  * Easy to maintain
    * Ability to replace parts
    * Ability to troubleshoot


* Nice to have
  * Ability to communicate error states to user and other rovers
    * Nearby humans that prevent movement
    * Lack of power
    * Damage to drive system
    * Instability
  * Carry a load that is too long for a single rover, use cooperation with one other rover
    * Maximum weight: 25.0kg * number of rovers
  * Provide attachment points for the load
  * Carry tall loads that require a highly stable platform to be moved -> Ability to widen the rover by pushing the sides (including wheels) outwards
  * Carry loads wider than the rover
  * Ability to automatically connect to the charge point
  * Carry heavier loads
    * 100.0 kg

* Won't have
  * The rover will not be self loading or self unloading. It is assumed that there is a way
    to load the cargo on to the rover(s)
  * Automatic cargo strapping / clamping
  * High speed transport, maximum speed is limited to walking speed (1.5 m/s)
  * Traverse steep terrain, maximum hill angle 20 degrees
  * Terrain side slope no more than half the wheel diameter over the width of the rover
  * Terrain bumpy-ness will be less than half the wheel diameter high
