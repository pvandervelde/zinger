import math
import numpy

gravitational_acceleration = 9.80665
air_density = 1.225

def force_aerodynamic_friction(area_in_msq, drag_coefficient, velocity_in_meters_per_second):
    return 0.5 * air_density * area_in_msq * drag_coefficient * math.pow(velocity_in_meters_per_second, 2)

def force_rolling_friction(mass_in_kg, coefficient_of_friction, slope_angle):
    return coefficient_of_friction * force_gravity_normal_to_surface(mass_in_kg, slope_angle)

def force_gravity_normal_to_surface(mass_in_kg, slope_angle):
    return mass_in_kg * gravitational_acceleration * math.cos(math.radians(slope_angle))

def force_gravity_parallel_to_surface(mass_in_kg, slope_angle):
    return mass_in_kg * gravitational_acceleration * math.sin(math.radians(slope_angle))

def velocity_for_slope_and_power(power, area_in_msq, drag_coefficient, mass_in_kg, coefficient_of_friction, slope_angle):
    # F_aero * V + F_roll * V + F_hill * V - P = 0
    coeff_cube = 0.5 * air_density * area_in_msq * drag_coefficient
    coeff_square = 0.0
    coeff_lin = force_rolling_friction(mass_in_kg, coefficient_of_friction, slope_angle) + force_gravity_parallel_to_surface(mass_in_kg, slope_angle)

    roots = numpy.roots([coeff_cube, coeff_square, coeff_lin, -power])

    real_value = 0.0
    for i in range(len(roots)):
     if numpy.isreal(roots[i]):
         real_value = numpy.real(roots[i])

    return real_value

def wheel_load(
    area_in_msq,
    drag_coefficient,
    velocity_in_meters_per_second,
    mass_in_kg,
    coefficient_of_friction,
    slope_angle):
    force_air = force_aerodynamic_friction(
        area_in_msq,
        drag_coefficient,
        velocity_in_meters_per_second)

    force_roll = force_rolling_friction(
        mass_in_kg,
        coefficient_of_friction,
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
