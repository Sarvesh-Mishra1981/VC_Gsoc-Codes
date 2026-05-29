def main(inputs, outputs, parameters, synchronise):
    state = "FORWARD"

    forward_speed = 0.16
    turn_speed = 0.45

    front_stop_distance = 0.60
    front_clear_distance = 0.90

    while True:
        front_distance = getattr(inputs, "front_distance", 10.0)

        if front_distance is None:
            front_distance = 10.0

        linear_x = 0.0
        angular_z = 0.0

        if state == "FORWARD":
            if front_distance < front_stop_distance:
                state = "TURN_LEFT"
                linear_x = 0.0
                angular_z = turn_speed
            else:
                linear_x = forward_speed
                angular_z = 0.0

        elif state == "TURN_LEFT":
            linear_x = 0.0
            angular_z = turn_speed

            if front_distance > front_clear_distance:
                state = "FORWARD"
                linear_x = forward_speed
                angular_z = 0.0

        outputs.linear_x = linear_x
        outputs.angular_z = angular_z
        outputs.state = state

        synchronise()