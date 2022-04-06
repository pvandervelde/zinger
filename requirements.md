# Requirements

This document records the specifications for the CrateRover.


## Purpose

The purpose for CrateRover is to liberate humans from having to carry heavy or unwieldy loads over
semi-flat terrain.


* define
  * heavy
  * unwieldy
  * semi-flat terrain
    * Material
    * slope
    * roughness


* Must have
  * Carry a load
    * One that fits the bed
      * Maximum weight: 25.0 kg
      * Size (l * w * h): 0.520m x 0.350m x 0.268m
    * Ones that don't fit the bed, use cooperation with other rovers
      * Maximum weight: 25.0kg * number of rovers
      * Determine how many rovers are necessary to carry a designated load
    * Provide attachment points for the load
  * Navigate terrain from current position to the pick-up location
  * Navigate terrain from the pick-up point to the delivery point
    * Taking into account the size and shape of the load
    * Communicate and coordinate between all the rovers involved
  * Communicate status and progress
  * Ability to turn on the spot
  * Ability to work for a 8 hour period

* Nice to have
  * Ability to widen the rover by pushing the sides (including wheels) outwards
  * Ability to automatically connect to the charge point

* Won't have
  * The rover will not be self loading or self unloading. It is assumed that there is a way
    to load the cargo on to the rover(s)
  * Automatic cargo strapping / clamping
  * High speed transport, maximum speed is limited to walking speed (1.5 m/s)
  * Traverse steep terrain, maximum hill angle 20 degrees
  * Terrain side slope no more than half the wheel diameter over the width of the rover
  * Terrain bumpy-ness will be less than half the wheel diameter high






* Movement
  * Rolling movement
  * Lights to indicate status


* Energy
  * Separate power pack for compute and drive
  * Automatic charging
  * m-stop (mobility-stop) with lights indicating status
    * Only kill motion, but not compute & sensors
    * When resetting the switch -> What is the state. Movement should be confirmed somehow
    * Apply brakes
    * Send signal to other robots that are in the same cargo transport
    * Wireless m-stop -> On a separate channel with constant heartbeats.
    * https://www.fortrobotics.com/wireless-emergency-stop
  * Pause button - Stops robot but maintains current task list
  * hot glue over exposed contacts to prevent shorts (black hot glue is better)

* Intelligence


* Sensing
  * Detection of humans nearby. Ensures that we stop before danger occurs
    * how do we deal with humans in between two rovers that are part of the same load

* Materials
 * Light
 * Strong
 * Easy to acquire
 * Easy to work with

* Construction methods
  * Ease of construction
  * Ease of maintenance
    * Ability to replace parts
    * Ability to troubleshoot
