#!/usr/bin/env python3


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def main(inputs, outputs, parameters, synchronise):
    filtered_v = 0.0
    filtered_w = 0.0

    max_speed = 2.0
    min_speed = 0.4
    max_turn = 0.8

    alpha = 0.20

    while True:
        try:
            car_force = inputs.read_array("CarForce")
            obs_force = inputs.read_array("ObsForce")
            distance_data = inputs.read_number("TargetDistance")

            target_v = 0.0
            target_w = 0.0

            if (
                car_force is not None
                and obs_force is not None
                and distance_data is not None
            ):
                avg_force_x = float(car_force[0]) + float(obs_force[0])
                avg_force_y = float(car_force[1]) + float(obs_force[1])

                distance_to_target = float(distance_data[0])

                turn_strength = abs(avg_force_y)

                target_v = max_speed - 0.35 * turn_strength
                target_v = clamp(target_v, min_speed, max_speed)

                target_w = avg_force_y * 0.25
                target_w = clamp(target_w, -max_turn, max_turn)

                if distance_to_target < 1.0:
                    target_v = 0.0
                    target_w = 0.0

            filtered_v = (1.0 - alpha) * filtered_v + alpha * target_v
            filtered_w = (1.0 - alpha) * filtered_w + alpha * target_w

            outputs.share_array(
                "VelsOut",
                [
                    filtered_v,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    filtered_w,
                ],
            )

        except Exception as error:
            print("Velocity Mixer error:", error)
            outputs.share_array(
                "VelsOut",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )

        synchronise()