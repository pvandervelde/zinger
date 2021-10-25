import math
import numpy
import pandas as pd

from crate_rover_specs import *

air_density = 1.225

class Physics:
    def coefficient_of_aerodynamic_drag():
        return 1.00 # move this to a different class. It's not a physical property

    def coefficient_of_friction():
        return 0.25 # move this to a different class. It's not a physical property

    def gravitational_acceleration():
        return 9.80665

    def force_aerodynamic_friction(area_in_msq, velocity_in_meters_per_second):
        return 0.5 * air_density * area_in_msq * Physics.coefficient_of_aerodynamic_drag() * math.pow(velocity_in_meters_per_second, 2)

    def force_rolling_friction(mass_in_kg, slope_angle):
        return Physics.coefficient_of_friction() * Physics.force_gravity_normal_to_surface(mass_in_kg, slope_angle)

    def force_gravity_normal_to_surface(mass_in_kg, slope_angle):
        return mass_in_kg * Physics.gravitational_acceleration() * math.cos(math.radians(slope_angle))

    def force_gravity_parallel_to_surface(mass_in_kg, slope_angle):
        return mass_in_kg * Physics.gravitational_acceleration() * math.sin(math.radians(slope_angle))

# Coefficient of friction depending on ground material (based on material from here: https://www.roboteq.com/roboamr-2021)
#
# Surface type                          #       #       #
# Concrete (good / average / bad)       0.010   0.015   0.020
# Asphalt (good / average / bad)        0.012   0.017   0.022
# Wood (dry / dusty / wet)              0.010   0.005   0.001
# Snow (50mm / 100mm)                   0.025   0.037   -
# Dirt (smooth / sandy)                 0.025   0.037   -
# Mud (firm / medium / soft)            0.037   0.090   0.150
# Grass (firm / soft)                   0.055   0.075   -
# Sand (firm / soft / dune)             0.060   0.150   0.300

class Rover:
    ''' Stores information about the Rover. '''

    def __init__(self, specs: CrateRoverSpecifications):
        self.specs = specs

        self.chassis_cog = self.specs.chassis.center_of_gravity()
        self.cargo_cog = self.specs.cargo.center_of_gravity()

        self.rover_cog = Coordinate(
            self.chassis_cog.x,
            self.chassis_cog.y,
            (self.chassis_cog.z * self.specs.chassis.weight_in_kg + (self.specs.chassis.bounding_box.height_in_meters + self.cargo_cog.z) * self.specs.cargo.maximum_weight_in_kg) / (self.specs.chassis.weight_in_kg + self.specs.cargo.maximum_weight_in_kg))

    def center_of_gravity(self) -> Coordinate:
        return self.rover_cog

    def frontal_surface(self) -> float:
        return self.specs.chassis.bounding_box.width_in_meters * (self.specs.chassis.bounding_box.height_in_meters + self.specs.cargo.bounding_box.height_in_meters)

    def weight_of_rover_chassis_in_kg(self) -> float:
        return self.specs.specs.chassis.weight_in_kg

    def weight_with_cargo_in_kg(self) -> float:
        return self.specs.chassis.weight_in_kg + self.specs.cargo.maximum_weight_in_kg

    def velocity_for_slope_and_power(self, power, area_in_msq, mass_in_kg, slope_angle):
        # F_aero * V + F_roll * V + F_hill * V - P = 0
        coeff_cube = 0.5 * air_density * area_in_msq * Physics.coefficient_of_aerodynamic_drag()
        coeff_square = 0.0
        coeff_lin = Physics.force_rolling_friction(mass_in_kg, slope_angle) + Physics.force_gravity_parallel_to_surface(mass_in_kg, slope_angle)

        roots = numpy.roots([coeff_cube, coeff_square, coeff_lin, -power])

        real_value = 0.0
        for i in range(len(roots)):
            if numpy.isreal(roots[i]):
                real_value = numpy.real(roots[i])

        return real_value

    def corner_radius_for_velocity_and_angle(self, velocity: float, cog_y: float, cog_z: float, slope_angle: float):
        '''
            Determines the minimum turn radius for a given velocity, ground angle
            see: https://core.ac.uk/download/pdf/147124666.pdf
        '''
        coeff = math.pow(velocity, 2.0) / Physics.gravitational_acceleration()

        angle = math.tan(math.radians(slope_angle))

        ssf = cog_y / cog_z

        return coeff * ((ssf * angle + 1) / (ssf - angle))

    def wheel_force_and_power_with_cargo(
        self,
        velocity_in_meters_per_second,
        slope_angle):
        force_air = Physics.force_aerodynamic_friction(
            self.frontal_surface(),
            velocity_in_meters_per_second)

        force_roll = Physics.force_rolling_friction(
            self.weight_with_cargo_in_kg(),
            slope_angle)

        force_slope = Physics.force_gravity_parallel_to_surface(
            self.weight_with_cargo_in_kg(),
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
