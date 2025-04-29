def line_exists(line_sensor_left, line_sensor_right, cutoff=150):
    """
    If both sensors detect a line, we should stop the robot because it means it has passed the line intersection.
    """
    r1 = line_sensor_left.reflectivity()
    r2 = line_sensor_right.reflectivity()
    if(r1 < cutoff and r2 < cutoff):
        return False
    return True
