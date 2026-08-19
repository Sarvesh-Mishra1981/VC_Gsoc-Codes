#!/usr/bin/env python3

import random


def main(inputs, outputs, parameters, synchronise):
    FORWARD = 0.0
    TURN_LEFT = 1.0
    TURN_RIGHT = 2.0
    UTURN_LEFT = 3.0
    UTURN_RIGHT = 4.0
    STOP = 5.0

    state = STOP
    turn_ticks = 0

    stop_distance = 0.60
    clear_distance = 0.90

    normal_turn_ticks = 40
    uturn_ticks = 80

    while True:
        distance_wire = inputs.read_number("In")
        ready_wire = inputs.read_number("Ready")

        ready = ready_wire is not None and float(ready_wire[0]) > 0.5

        if not ready or distance_wire is None:
            state = STOP
            turn_ticks = 0

        else:
            front_distance = float(distance_wire[0])

            if state == STOP:
                state = FORWARD

            if state == FORWARD:
                if front_distance < stop_distance:
                    state = random.choice([
                        TURN_LEFT,
                        TURN_RIGHT,
                        TURN_LEFT,
                        TURN_RIGHT,
                        UTURN_LEFT,
                        UTURN_RIGHT,
                    ])

                    if state in [UTURN_LEFT, UTURN_RIGHT]:
                        turn_ticks = uturn_ticks
                    else:
                        turn_ticks = normal_turn_ticks

            elif state in [TURN_LEFT, TURN_RIGHT, UTURN_LEFT, UTURN_RIGHT]:
                turn_ticks -= 1

                if turn_ticks <= 0 and front_distance > clear_distance:
                    state = FORWARD

                elif turn_ticks <= -30:
                    state = FORWARD

        outputs.share_number("Out", state)

        synchronise()