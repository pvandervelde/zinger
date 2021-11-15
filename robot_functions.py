from abc import abstractmethod
import math
import numpy
import pandas as pd

from crate_rover_specs import *

air_density = 1.225

shaft_diameters = [0.006, 0.008, 0.010, 0.012, 0.014, 0.016, 0.018, 0.020]

class Material:
    def __init__(self, name: str, yield_strength: float, shear_modulus: float):
        self.name = name
        self.yield_strength = yield_strength
        self.shear_modulus = shear_modulus

materials = [
    Material('Stainless steel', 205e6, 81e9),
    Material('Aluminium 2024', 97e6, 28e9),
]

class Engineering:
    def safety_factor_propulsion() -> float:
        return 1.25

    def safety_factor_structural() -> float:
        return 2.0

class Physics:
    def coefficient_of_aerodynamic_drag() -> float:
        return 1.00 # move this to a different class. It's not a physical property

    def coefficient_of_friction() -> float:
        return 0.25 # move this to a different class. It's not a physical property

    def gravitational_acceleration() -> float:
        return 9.80665

    def force_aerodynamic_friction(area_in_msq: float, velocity_in_meters_per_second: float) -> float:
        return 0.5 * air_density * area_in_msq * Physics.coefficient_of_aerodynamic_drag() * math.pow(velocity_in_meters_per_second, 2)

    def force_rolling_friction(mass_in_kg: float, slope_angle: float) -> float:
        return Physics.coefficient_of_friction() * Physics.force_gravity_normal_to_surface(mass_in_kg, slope_angle)

    def force_gravity_normal_to_surface(mass_in_kg: float, slope_angle: float) -> float:
        return mass_in_kg * Physics.gravitational_acceleration() * math.cos(math.radians(slope_angle))

    def force_gravity_parallel_to_surface(mass_in_kg: float, slope_angle: float) -> float:
        return mass_in_kg * Physics.gravitational_acceleration() * math.sin(math.radians(slope_angle))

class Friction:
    @abstractmethod
    def kinematic(self, material: str, condition: str) -> float:
        pass

    @abstractmethod
    def rolling(self, material: str, condition: str) -> float:
        pass

    @abstractmethod
    def static(self, material: str, condition: str) -> float:
        pass

class FrictionForRubber(Friction):
    # Coefficient of friction depending on ground material (based on material from here: https://www.roboteq.com/roboamr-2021)
    coefficients_rolling = {
        "concrete-dry": 0.010,
        "concrete-wet": 0.010,
        "asphalt-dry": 0.012,
        "asphalt-wet": 0.012,
        "wood-dry": 0.010,
        "wood-wet": 0.001,
        "dirt-smooth": 0.025,
        "dirt-sandy": 0.037,
        "mud-firm": 0.037,
        "mud-medium": 0.090,
        "mud-soft": 0.150,
        "grass-firm": 0.055,
        "grass-soft": 0.075,
        "sand-firm": 0.060,
        "sand-soft": 0.150,
        "sand-dune": 0.300
    }

    # See here: https://hypertextbook.com/facts/2006/MatthewMichaels.shtml
    coefficients_static = {
        "concrete-dry": 1.00,
        "concrete-wet": 0.30
    }

    def kinematic(self, material: str, condition: str) -> float:
        return 1.0

    def rolling(self, material: str, condition: str) -> float:
        return self.coefficients_rolling[f'{material}-{condition}']

    def static(self, material: str, condition: str) -> float:
        return self.coefficients_static[f'{material}-{condition}']

# Need some way to store material properties

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

    def velocity_for_slope_and_power(self, power: float, area_in_msq: float, mass_in_kg: float, slope_angle: float) -> float:
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

    def corner_radius_for_velocity_and_angle(self, velocity: float, cog_y: float, cog_z: float, slope_angle: float) -> float:
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
        velocity_in_meters_per_second: float,
        slope_angle: float) -> float:
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


def maximum_shear_stress(outer_radius: float, inner_radius: float, torque: float) -> float:
    # Tmax = (π / 16) τmax (D4 - d4) / D
    # (16 Tmax / π) D / (D^4 - d^4) = τmax
    return 16 * torque * outer_radius * 2.0 / (math.pi * (math.pow(outer_radius * 2, 4.0) - math.pow(inner_radius * 2, 4.0)))

def polar_moment_of_inertia(outer_radius: float, inner_radius: float) -> float:
    # J = π (D^4 - d^4) / 32
    return math.pi * (math.pow(outer_radius * 2, 4.0) - math.pow(inner_radius * 2, 4.0)) / 32

def maximum_torque_for_shaft(outer_radius: float, inner_radius: float, maximum_shear_stress: float) -> float:
    # Tmax = (π / 16) τmax (D^4 - d^4) / D
    return (math.pi / 16) * maximum_shear_stress * (math.pow(outer_radius * 2, 4.0) - math.pow(inner_radius * 2, 4.0)) / (outer_radius * 2)

def torsional_deflection_in_radians_of_shaft(outer_radius: float, inner_radius: float, length: float, shear_modulus: float, torque: float) -> float:
    # α = 32 L T / (G π (D^4- d^4))
    return 32 * length * torque / (shear_modulus * math.pi * (math.pow(outer_radius * 2, 4.0) - math.pow(inner_radius * 2, 4.0)))

def torsional_deflection_in_degrees_of_shaft(outer_radius: float, inner_radius: float, length: float, shear_modulus: float, torque: float) -> float:
    # α = 32 L T / (G π (D^4- d^4))
    deflection_in_radians = torsional_deflection_in_radians_of_shaft(outer_radius, inner_radius, length, shear_modulus, torque)
    return deflection_in_radians * 180.0 / math.pi

def minimum_diameter_for_solid_shaft(torque: float, maximum_shear_stress: float) -> float:
    # D = 1.72 (Tmax / τmax)^(1/3)
    return 1.72 * math.pow(torque / maximum_shear_stress, 1.0 / 3.0)

def find_solid_shaft_diameter_for_torque(torque: float, maximum_deflection_in_degrees: float, yield_strength: float, shear_modulus: float) -> float:
    selected_diameter = shaft_diameters[-1]
    for diameter in shaft_diameters:
        max_shear_stress = maximum_shear_stress(diameter / 2.0, 0.0, torque)
        if max_shear_stress > yield_strength:
            continue

        deflection = torsional_deflection_in_degrees_of_shaft(diameter / 2.0, 0.0, 1.0, shear_modulus, torque)
        if deflection > maximum_deflection_in_degrees:
            continue

        selected_diameter = diameter
        break

    return selected_diameter

