import numpy as np


class LowPassFilter:

    def __init__(self):
        # set some useful constants
        self.memorySize = 5
        self.buffer = []

    def add_measure(self, measure):
        """
        Manage buffer
        """
        self.buffer = self.buffer[1 - self.memorySize:] + [measure]

    def clear_buffer(self):
        """
        Clear buffer
        """
        self.buffer = []

    def median_filter(self):
        """
        Take median value of the self.memorySize latest elements of the buffer
        """
        return np.median(self.buffer[-self.memorySize:])
