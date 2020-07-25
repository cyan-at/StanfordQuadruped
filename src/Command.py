import numpy as np


class Command(object):
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = -0.16
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        
        self.hop_event = False
        self.trot_event = False
        self.activate_event = False

    def __str__(self):
        s = "-------------------\n"
        s += "horizontal_velocity: %.3f, %.3f\n" % (
            self.horizontal_velocity[0], self.horizontal_velocity[1])
        s += "yaw_rate: %.3f\n" % (self.yaw_rate)
        s += "height: %.3f\n" % (self.height)
        s += "pitch: %.3f\n" % (self.pitch)
        s += "roll: %.3f\n" % (self.roll)
        s += "activation: %d\n" % (self.activation)
        s += "hop_event: %d\n" % (self.hop_event)
        s += "trot_event: %d\n" % (self.trot_event)
        s += "activate_event: %d\n" % (self.activate_event)
        return s
