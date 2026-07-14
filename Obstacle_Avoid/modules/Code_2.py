#!/usr/bin/env python3

import math


def safe_float(value):
    try:
        number = float(value)
        if math.isfinite(number):
            return number
    except Exception:
        pass

    return None


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def main(inputs, outputs, parameters, synchronise):
    filtered_x = 0.0
    filtered_y = 0.0

    obstacle_weight = 4.0
    max_force = 6.0
    alpha = 0.25

    while True:
        try:
            laser_data = inputs.read_array("LaserIn")

            obs_x = 0.0
            obs_y = 0.0

            if laser_data is not None:
                laser_count = len(laser_data)

                if laser_count > 1:
                    for index in range(laser_count):
                        dist = safe_float(laser_data[index])

                        if dist is None:
                            continue

                        if dist < 0.20 or dist > 3.0:
                            continue

                        angle = (
                            -math.pi / 2.0
                            + index * (math.pi / float(laser_count - 1))
                        )

                        strength = (3.0 - dist) / 3.0
                        repulsive_magnitude = strength * strength

                        obs_x -= repulsive_magnitude * math.cos(angle)
                        obs_y -= repulsive_magnitude * math.sin(angle)

                obs_x *= obstacle_weight
                obs_y *= obstacle_weight

                obs_x = clamp(obs_x, -max_force, max_force)
                obs_y = clamp(obs_y, -max_force, max_force)

                filtered_x = (1.0 - alpha) * filtered_x + alpha * obs_x
                filtered_y = (1.0 - alpha) * filtered_y + alpha * obs_y

            outputs.share_array("ObsForce", [filtered_x, filtered_y])

        except Exception as error:
            print("Obstacle Force error:", error)
            outputs.share_array("ObsForce", [0.0, 0.0])

        synchronise()