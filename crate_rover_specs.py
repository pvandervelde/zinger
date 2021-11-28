# imports
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class Coordinate:

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

class BoundingBox:

    def __init__(self, length_in_meters: float, width_in_meters: float, height_in_meters: float):
        self.length_in_meters = length_in_meters
        self.width_in_meters = width_in_meters
        self.height_in_meters = height_in_meters

    def center_of_gravity(self) -> Coordinate:
        return Coordinate(self.length_in_meters / 2.0, self.width_in_meters / 2.0, self.height_in_meters / 2.0)

class Cargo:
    ''' Stores information about an item of cargo transported by the CrateRover. '''

    def __init__(self, bounding_box: BoundingBox, maximum_weight_in_kg: float):
        self.bounding_box = bounding_box
        self.maximum_weight_in_kg = maximum_weight_in_kg

    def center_of_gravity(self) -> Coordinate:
        return self.bounding_box.center_of_gravity()

    def to_dataframe(self) -> pd.DataFrame:
        ''' Prints a table of information describing the Cargo. '''
        crate_data = [["Maximum weight (kg)", self.maximum_weight_in_kg],
              ["Length (m)", self.bounding_box.length_in_meters],
              ["Width (m)", self.bounding_box.width_in_meters],
              ["Height (m)", self.bounding_box.height_in_meters]]
        return pd.DataFrame(crate_data, columns=["Cargo", "Value"])

class RoverWheel:

    def __init__(self, radius_in_meters: float):
        self.radius_in_meters = radius_in_meters

class RoverChassis:

    def __init__(self, bounding_box: BoundingBox, weight_in_kg: float, number_of_wheels: int, wheel: RoverWheel):
        self.bounding_box = bounding_box
        self.weight_in_kg = weight_in_kg
        self.number_of_wheels = number_of_wheels
        self.wheel = wheel

    def center_of_gravity(self) -> Coordinate:
        return self.bounding_box.center_of_gravity()

class RoverPerformance:

    def __init__(
        self,
        min_velocity_on_flat_ground_in_meterspersecond: float,
        min_velocity_on_sloped_ground_in_meterspersecond: float,
        maximum_slope_angle_with_maximum_load: float,
        maximum_slope_angle_without_load: float,
        minimum_battery_life_in_hours: float
    ):
        self.min_velocity_on_flat_ground = min_velocity_on_flat_ground_in_meterspersecond
        self.min_velocity_on_sloped_ground = min_velocity_on_sloped_ground_in_meterspersecond
        self.max_slope_angle_with_maximum_load = maximum_slope_angle_with_maximum_load
        self.max_slope_angle_without_load = maximum_slope_angle_without_load
        self.minimum_battery_life_in_hours = minimum_battery_life_in_hours

    def maximum_slope_angle_with_maximum_load(self) -> float:
        return self.max_slope_angle_with_maximum_load

    def maximum_slope_angle_without_load(self) -> float:
        return self.max_slope_angle_without_load

    def maximum_velocity_on_flat_ground(self) -> float:
        return self.min_velocity_on_flat_ground

    def maximum_velocity_on_sloped_ground(self) -> float:
        return self.min_velocity_on_sloped_ground

    def to_dataframe(self) -> pd.DataFrame:
        performance_data = [
            ["Minimum speed on the flat with load (m/s)", self.min_velocity_on_flat_ground],
            ["Minimum speed on a slope with load (m/s)", self.min_velocity_on_flat_ground],
            ["Maximum hill angle without load (degrees)", self.max_slope_angle_without_load],
            ["Maximum hill angle with load (degrees)", self.max_slope_angle_with_maximum_load],
            ["Minimum battery life (hours)", self.minimum_battery_life_in_hours]
        ]
        return pd.DataFrame(performance_data, columns=["Performance", "Value"])

class CrateRoverSpecifications:

    def __init__(
        self,
        cargo: Cargo,
        expectedPerformance: RoverPerformance,
        chassis: RoverChassis):
        self.cargo = cargo
        self.performance = expectedPerformance
        self.chassis = chassis

def RoverSpecification() -> CrateRoverSpecifications:
    cargo = Cargo(
        BoundingBox(
            0.520,
            0.350,
            0.268),
        50.0)

    performance = RoverPerformance(
        2.0,
        2.0,
        15.0,
        30.0,
        5.0)

    rover_wheel = RoverWheel(0.075)
    rover_chassis = RoverChassis(
        BoundingBox(0.6, cargo.bounding_box.width_in_meters + 0.05, rover_wheel.radius_in_meters * 2 + 0.10),
        20.0,
        4,
        rover_wheel)

    return CrateRoverSpecifications(cargo, performance, rover_chassis)