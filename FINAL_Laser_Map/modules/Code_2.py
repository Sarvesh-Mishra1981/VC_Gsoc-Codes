def main(inputs, outputs, parameters, synchronise):
    FORWARD = 0.0
    TURN_LEFT = 1.0

    state = FORWARD

    front_stop_distance = 0.60
    front_clear_distance = 0.90

    while True:
        front_wire = inputs.read_number("front_distance")

        if front_wire is None:
            front = 10.0
        else:
            front = float(front_wire[0])

        if state == FORWARD:
            if front < front_stop_distance:
                state = TURN_LEFT

        elif state == TURN_LEFT:
            if front > front_clear_distance:
                state = FORWARD

        outputs.share_number("state_code", state)

        synchronise()