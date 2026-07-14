#!/usr/bin/env python3

import math


def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty

    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)

    return x_rel, y_rel


def main(inputs, outputs, parameters, synchronise):
    last_car_force = [0.0, 0.0]

    while True:
        odom_data = inputs.read_array("OdomIn")
        target_data = inputs.read_array("TargetIn")

        if odom_data is not None and target_data is not None:
            robotx = float(odom_data[0])
            roboty = float(odom_data[1])
            robott = float(odom_data[2])

            target_abs_x = float(target_data[0])
            target_abs_y = float(target_data[1])

            target_rel_x, target_rel_y = absolute2relative(
                target_abs_x,
                target_abs_y,
                robotx,
                roboty,
                robott,
            )

            target_weight = 1.0

            last_car_force = [
                target_rel_x * target_weight,
                target_rel_y * target_weight,
            ]

        outputs.share_array("CarForce", last_car_force)

        synchronise()