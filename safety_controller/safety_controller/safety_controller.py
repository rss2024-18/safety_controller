import numpy as np

def safety_control(self, msg):
    """
    Compute distance to front wall and stop the car if the wall is too close
    """
    # distance from wall to stop the car
    distance_needed_to_stop = self.VELOCITY * 1/3
    # LIDAR scans from the front of the car
    front_data = np.array(self.return_data(msg.ranges, 1))
    # Angle values of the lidar scans
    middle_angles = np.array(np.cos(self.return_data(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment).tolist(), 1)))
    # scaled distance due to angle
    true_dist = np.multiply(middle_angles, front_data)
    if np.median(true_dist) < distance_needed_to_stop:
        return 0.0
    return self.VELOCITY

def return_data(self, data, flag = 0):
    """
    Returns appropriate data slice for side and speed
    """
    data = self.slice_scan(data)
    if flag:
        return np.array(data["ms"])
    if self.SIDE == -1:
        return np.array(data["brs"])
    return np.flip(np.array(data["bls"]))

def slice_scan(self, ranges):
    """
    slice scan into managable pieces a
    """
    back_front_divider = int(len(ranges) * 16/33)
    front_middle_divider = int(len(ranges) * 16/33)
    total_length = int(len(ranges))
    bls = ranges[total_length - back_front_divider:total_length]
    ms = ranges[front_middle_divider:total_length - front_middle_divider]
    brs = ranges[0:back_front_divider]
    return {'brs': brs, "ms": ms, "bls" : bls}