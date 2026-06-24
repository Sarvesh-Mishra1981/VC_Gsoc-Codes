def main(inputs, outputs, parameters, synchronise):
    FORWARD = 0.0
    TURN_LEFT = 1.0

    forward_speed = 0.70
    turn_speed = 1.25

    while True:
        state_wire = inputs.read_number("state_code")

        if state_wire is None:
            state = FORWARD
        else:
            state = float(state_wire[0])

        if state == FORWARD:
            linear_x = forward_speed
            angular_z = 0.0
        elif state == TURN_LEFT:
            linear_x = 0.0
            angular_z = turn_speed
        else:
            linear_x = 0.0
            angular_z = 0.0

        outputs.share_number("linear_x", linear_x)
        outputs.share_number("angular_z", angular_z)

        synchronise()