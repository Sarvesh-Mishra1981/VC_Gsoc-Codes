def main(inputs, outputs, parameters, synchronise):
    FORWARD = 0.0
    TURN_LEFT = 1.0
    STOP = 2.0

    state = STOP

    stop_distance = 0.60
    clear_distance = 0.90

    while True:
        distance_wire = inputs.read_number("front_distance")
        ready_wire = inputs.read_number("laser_ready")

        laser_ready = (
            ready_wire is not None
            and float(ready_wire[0]) > 0.5
        )

        if not laser_ready or distance_wire is None:
            state = STOP
        else:
            front_distance = float(distance_wire[0])

            if state == STOP:
                state = FORWARD

            if state == FORWARD:
                if front_distance < stop_distance:
                    state = TURN_LEFT

            elif state == TURN_LEFT:
                if front_distance > clear_distance:
                    state = FORWARD

        outputs.share_number("state_code", state)
        synchronise()