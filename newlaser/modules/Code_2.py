import math


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
    front_width = 0.35

    while True:
        laser_ready = read_input(inputs, "laser_ready", False)
        ranges = read_input(inputs, "ranges", [])
        angle_min = read_input(inputs, "angle_min", 0.0)
        angle_increment = read_input(inputs, "angle_increment", 0.0)

        front_distance = 10.0

        if laser_ready and ranges is not None:
            values = []

            for i, dist in enumerate(ranges):
                angle = angle_min + i * angle_increment

                if -front_width <= angle <= front_width:
                    if math.isfinite(dist) and dist > 0.05:
                        values.append(dist)

            if values:
                front_distance = min(values)

        write_output(outputs, "front_distance", front_distance)

        synchronise()