import math
import numpy
import pandas as pd

air_density = 1.225

class Physics:
    def coefficient_of_aerodynamic_drag():
        return 1.00 # move this to a different class. It's not a physical property

    def coefficient_of_friction():
        return 0.25 # move this to a different class. It's not a physical property

    def gravitational_acceleration():
        return 9.80665

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

class RoverPerformance:

    def __init__(
        self,
        max_velocity_in_meterspersecond: float,
        maximum_slope_angle_with_maximum_load: float,
        maximum_slope_angle_without_load: float,
        minimum_battery_life_in_hours: float
    ):
        self.max_velocity = max_velocity_in_meterspersecond
        self.max_slope_angle_with_maximum_load = maximum_slope_angle_with_maximum_load
        self.max_slope_angle_without_load = maximum_slope_angle_without_load
        self.minimum_battery_life_in_hours = minimum_battery_life_in_hours

    def maximum_slope_angle_with_maximum_load(self) -> float:
        return self.max_slope_angle_with_maximum_load

    def maximum_slope_angle_without_load(self) -> float:
        return self.max_slope_angle_without_load

    def maximum_velocity_on_flat_ground(self) -> float:
        return self.max_velocity

    def to_dataframe(self) -> pd.DataFrame:
        performance_data = [
            ["Maximum speed on the flat (m/s)", self.max_velocity],
            ["Maximum hill angle without load (degrees)", self.max_slope_angle_without_load],
            ["Maximum hill angle with load (degrees)", self.max_slope_angle_with_maximum_load],
            ["Minimum battery life (hours)", self.minimum_battery_life_in_hours]
        ]
        return pd.DataFrame(performance_data, columns=["Performance", "Value"])

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

class Rover:
    ''' Stores information about the Rover. '''

    def __init__(self, chassis: RoverChassis, cargo: Cargo):
        self.chassis = chassis
        self.cargo = cargo

    def center_of_gravity(self) -> Coordinate:
        chassis_cog = self.chassis.center_of_gravity()
        cargo_cog = self.cargo.center_of_gravity()

        return Coordinate(
            chassis_cog.x,
            chassis_cog.y,
            (chassis_cog.z * self.chassis.weight_in_kg + (self.chassis.bounding_box.height_in_meters + cargo_cog.z) * self.cargo.maximum_weight_in_kg) / (self.chassis.weight_in_kg + self.cargo.maximum_weight_in_kg))

    def frontal_surface(self) -> float:
        return self.chassis.bounding_box.width_in_meters * (self.chassis.bounding_box.height_in_meters + self.cargo.bounding_box.height_in_meters)

    def weight_empty_in_kg(self) -> float:
        return self.chassis.weight_in_kg

    def weight_load_in_kg(self) -> float:
        return self.chassis.weight_in_kg + self.cargo.maximum_weight_in_kg


def force_aerodynamic_friction(area_in_msq, velocity_in_meters_per_second):
    return 0.5 * air_density * area_in_msq * Physics.coefficient_of_aerodynamic_drag() * math.pow(velocity_in_meters_per_second, 2)

def force_rolling_friction(mass_in_kg, slope_angle):
    return Physics.coefficient_of_friction() * force_gravity_normal_to_surface(mass_in_kg, slope_angle)

def force_gravity_normal_to_surface(mass_in_kg, slope_angle):
    return mass_in_kg * Physics.gravitational_acceleration() * math.cos(math.radians(slope_angle))

def force_gravity_parallel_to_surface(mass_in_kg, slope_angle):
    return mass_in_kg * Physics.gravitational_acceleration() * math.sin(math.radians(slope_angle))

def velocity_for_slope_and_power(power, area_in_msq, mass_in_kg, slope_angle):
    # F_aero * V + F_roll * V + F_hill * V - P = 0
    coeff_cube = 0.5 * air_density * area_in_msq * Physics.coefficient_of_aerodynamic_drag()
    coeff_square = 0.0
    coeff_lin = force_rolling_friction(mass_in_kg, slope_angle) + force_gravity_parallel_to_surface(mass_in_kg, slope_angle)

    roots = numpy.roots([coeff_cube, coeff_square, coeff_lin, -power])

    real_value = 0.0
    for i in range(len(roots)):
     if numpy.isreal(roots[i]):
         real_value = numpy.real(roots[i])

    return real_value

def corner_radius_for_velocity_and_angle(velocity: float, cog_y: float, cog_z: float, slope_angle: float):
    '''
        Determines the minimum turn radius for a given velocity, ground angle
        see: https://core.ac.uk/download/pdf/147124666.pdf
    '''
    coeff = math.pow(velocity, 2.0) / Physics.gravitational_acceleration()

    angle = math.tan(math.radians(slope_angle))

    ssf = cog_y / cog_z

    return coeff * ((ssf * angle + 1) / (ssf - angle))

def wheel_load(
    area_in_msq,
    velocity_in_meters_per_second,
    mass_in_kg,
    slope_angle):
    force_air = force_aerodynamic_friction(
        area_in_msq,
        velocity_in_meters_per_second)

    force_roll = force_rolling_friction(
        mass_in_kg,
        slope_angle)

    force_slope = force_gravity_parallel_to_surface(
        mass_in_kg,
        slope_angle
    )

    wheel_force = force_air + force_roll + force_slope
    power = wheel_force * velocity_in_meters_per_second

    return wheel_force, power






def electrical_efficiency(battery_to_controller_efficiency,
                          controller_to_motor_efficiency, motor_efficiency,
                          motor_gear_efficiency, number_of_gears):
    result = battery_to_controller_efficiency * controller_to_motor_efficiency * motor_efficiency * math.pow(
        motor_gear_efficiency, number_of_gears)

    return result
