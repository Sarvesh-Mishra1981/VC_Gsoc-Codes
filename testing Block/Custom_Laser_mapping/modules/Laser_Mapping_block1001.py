#!/usr/bin/env python3

import math


def main(inputs, outputs, parameters, synchronise):
    last_distance = 10.0
    ready = 0.0

    while True:
        sensor_data = inputs.read_array("In")

        if sensor_data is not None and len(sensor_data) > 0:
            values = []
            count = len(sensor_data)

            for index in range(count):
                try:
                    distance = float(sensor_data[index])
                except Exception:
                    continue

                if not math.isfinite(distance):
                    continue

                if distance <= 0.05:
                    continue

                values.append(distance)

            if values:
                last_distance = min(values)
                ready = 1.0

        outputs.share_number("Out", last_distance)
        outputs.share_number("Ready", ready)

        synchronise()