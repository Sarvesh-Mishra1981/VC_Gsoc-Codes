#!/usr/bin/env python3


def main(inputs, outputs, parameters, synchronise):
    last_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    while True:
        car_force = inputs.read_array("CarForce")
        obs_force = inputs.read_array("ObsForce")

        if car_force is not None and obs_force is not None:
            avg_force_y = float(car_force[1]) + float(obs_force[1])

            v = 3.0
            w = avg_force_y * 0.18

            if w > 0.75:
                w = 0.75

            if w < -0.75:
                w = -0.75

            last_velocity = [
                v,
                0.0,
                0.0,
                0.0,
                0.0,
                w,
            ]

        outputs.share_array("VelsOut", last_velocity)

        synchronise()