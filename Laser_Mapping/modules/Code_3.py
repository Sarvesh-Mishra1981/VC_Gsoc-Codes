def main(inputs, outputs, parameters, synchronise):
    FORWARD = 0.0
    TURN_LEFT = 1.0
    TURN_RIGHT = 2.0
    UTURN_LEFT = 3.0
    UTURN_RIGHT = 4.0

    forward_speed = 0.70
    turn_speed = 1.25

    while True:
        state_wire = inputs.read_number("state_code")

        linear_x = 0.0
        angular_z = 0.0

        if state_wire is not None:
            state = float(state_wire[0])

            if state == FORWARD:
                linear_x = forward_speed
                angular_z = 0.0

            elif state == TURN_LEFT:
                linear_x = 0.0
                angular_z = turn_speed

            elif state == TURN_RIGHT:
                linear_x = 0.0
                angular_z = -turn_speed

            elif state == UTURN_LEFT:
                linear_x = 0.0
                angular_z = turn_speed

            elif state == UTURN_RIGHT:
                linear_x = 0.0
                angular_z = -turn_speed

        outputs.share_number("linear_x", linear_x)
        outputs.share_number("angular_z", angular_z)

        synchronise()