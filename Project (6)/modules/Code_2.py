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


def main(inputs, outputs, parameters, synchronise):
    last_obs_force = [0.0, 0.0]

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

                        if dist < 0.05 or dist > 3.0:
                            continue

                        angle = (
                            -math.pi / 2.0
                            + index * (math.pi / float(laser_count - 1))
                        )

                        repulsive_magnitude = 1.0 / (dist * dist)

                        obs_x -= repulsive_magnitude * math.cos(angle)
                        obs_y -= repulsive_magnitude * math.sin(angle)

                obstacle_weight = 12.0

                last_obs_force = [
                    obs_x * obstacle_weight,
                    obs_y * obstacle_weight,
                ]

            outputs.share_array("ObsForce", last_obs_force)

        except Exception as error:
            print("Obstacle Force error:", error)
            outputs.share_array("ObsForce", [0.0, 0.0])

        synchronise()