#!/usr/bin/env python3


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def main(inputs, outputs, parameters, synchronise):
    last_velocity = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    while True:
        try:
            car_force = inputs.read_array("CarForce")
            obs_force = inputs.read_array("ObsForce")
            distance_data = inputs.read_number("TargetDistance")

            if (
                car_force is not None
                and obs_force is not None
                and distance_data is not None
            ):
                avg_force_x = float(car_force[0]) + float(obs_force[0])
                avg_force_y = float(car_force[1]) + float(obs_force[1])

                distance_to_target = float(distance_data[0])

                linear_speed = 3.0
                angular_speed = avg_force_y * 0.35

                angular_speed = clamp(
                    angular_speed,
                    -1.2,
                    1.2,
                )

                if distance_to_target < 1.0:
                    linear_speed = 0.0
                    angular_speed = 0.0

                last_velocity = [
                    linear_speed,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    angular_speed,
                ]

            outputs.share_array("VelsOut", last_velocity)

        except Exception as error:
            print("Velocity Mixer error:", error)

            outputs.share_array(
                "VelsOut",
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )

        synchronise()