def read_input(inputs, name, default=None):
    if hasattr(inputs, "read"):
        return inputs.read(name)
    if hasattr(inputs, "get"):
        return inputs.get(name, default)
    try:
        return inputs[name]
    except Exception:
        return getattr(inputs, name, default)


def write_output(outputs, name, value):
    if hasattr(outputs, "write"):
        outputs.write(name, value)
        return
    if hasattr(outputs, "set"):
        outputs.set(name, value)
        return
    try:
        outputs[name] = value
        return
    except Exception:
        pass
    setattr(outputs, name, value)


def main(inputs, outputs, parameters, synchronise):
    state = "FORWARD"

    forward_speed = 0.16
    turn_speed = 0.45

    front_stop_distance = 0.60
    front_clear_distance = 0.90

    while True:
        front_distance = read_input(inputs, "front_distance", 10.0)

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

        write_output(outputs, "linear_x", linear_x)
        write_output(outputs, "angular_z", angular_z)
        write_output(outputs, "state", state)

        synchronise()