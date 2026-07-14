#!/usr/bin/env python3


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def main(inputs, outputs, parameters, synchronise):
    filtered_v = 0.0
    filtered_w = 0.0

    max_speed = 2.0
    min_speed = 0.6

    steering_gain = 0.35
    max_turn = 0.95

    steering_deadband = 0.35

    alpha_v = 0.20
    alpha_w = 0.35

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
                avg_force_y = float(car_force[1]) + float(obs_force[1])
                distance_to_target = float(distance_data[0])

                if abs(avg_force_y) < steering_deadband:
                    avg_force_y = 0.0

                target_w = avg_force_y * steering_gain
                target_w = clamp(target_w, -max_turn, max_turn)

                turn_strength = abs(target_w)

                target_v = max_speed - 0.6 * turn_strength
                target_v = clamp(target_v, min_speed, max_speed)

                if distance_to_target < 1.0:
                    target_v = 0.0
                    target_w = 0.0

            filtered_v = (1.0 - alpha_v) * filtered_v + alpha_v * target_v
            filtered_w = (1.0 - alpha_w) * filtered_w + alpha_w * target_w

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