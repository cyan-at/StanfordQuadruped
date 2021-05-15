import numpy as np


class Command(object):
    """Stores movement command
    """

    def __init__(self):
        self.hop_event = False
        self.trot_event = False
        self.activate_event = 0

        # gamepad BUTTONS
        # discrete interaction but need continuous propagation
        # so need to transmit delta to controller
        # for stream semantics
        self.height = -0.16
        self.height_delta = 0.0
        self.roll = 0.0
        self.roll_delta = 0.0

        # gamepad ROCKERs
        # continous interaction / continuous propagation
        # state is enough, no delta
        self.horizontal_velocity =\
            np.array([0, 0])
        self.yaw_rate = 0.0
        self.pitch = 0.0

    def __str__(self):
        s = "\n-------------------\n"
        s += "horizontal_velocity: %.3f, %.3f\n" % (
            self.horizontal_velocity[0], self.horizontal_velocity[1])
        s += "yaw_rate: %.3f\n" % (self.yaw_rate)
        s += "pitch: %.3f\n" % (self.pitch)
        s += "roll_delta: %.3f\n" % (self.roll_delta)
        s += "height_delta: %.3f\n" % (self.height_delta)
        s += "hop_event: %d\n" % (self.hop_event)
        s += "trot_event: %d\n" % (self.trot_event)
        s += "activate_event: %d\n" % (self.activate_event)
        return s
