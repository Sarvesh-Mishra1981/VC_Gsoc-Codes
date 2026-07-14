#!/usr/bin/env python3

import math


def main(inputs, outputs, parameters, synchronise):
    last_obs_force = [0.0, 0.0]

    while True:
        laser_data = inputs.read_array("LaserIn")

        obs_x = 0.0
        obs_y = 0.0

        if laser_data is not None:
            for i in range(len(laser_data)):
                try:
                    dist = float(laser_data[i])
                except Exception:
                    continue

                if not math.isfinite(dist):
                    continue

                if dist <= 0.0:
                    continue

                angle = math.radians(i - 90)

                if dist < 3.0:
                    repulsive_magnitude = 1.0 / (dist ** 2)

                    obs_x -= repulsive_magnitude * math.cos(angle)
                    obs_y -= repulsive_magnitude * math.sin(angle)

            obstacle_weight = 6.0

            last_obs_force = [
                obs_x * obstacle_weight,
                obs_y * obstacle_weight,
            ]

        outputs.share_array("ObsForce", last_obs_force)

        synchronise()