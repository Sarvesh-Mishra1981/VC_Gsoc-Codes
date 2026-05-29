import math


def main(inputs, outputs, parameters, synchronise):
    front_width = 0.35

    while True:
        laser_ready = getattr(inputs, "laser_ready", False)
        ranges = getattr(inputs, "ranges", [])
        angle_min = getattr(inputs, "angle_min", 0.0)
        angle_increment = getattr(inputs, "angle_increment", 0.0)

        front_distance = 10.0

        if laser_ready and ranges:
            values = []

            for i, dist in enumerate(ranges):
                angle = angle_min + i * angle_increment

                if -front_width <= angle <= front_width:
                    if math.isfinite(dist) and dist > 0.05:
                        values.append(dist)

            if values:
                front_distance = min(values)

        outputs.front_distance = front_distance

        synchronise()