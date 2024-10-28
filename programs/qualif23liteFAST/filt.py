import numpy as np


class LowPassFilter:

    def __init__(self):
        # set some useful constants
        self.memorySize = 3

        self.buffer = []
        self.history = []

        self.consign = None

    def set_consign(self, consign):
        """
        Define a consign to follow
        """
        self.consign = consign

    def clear_data(self):
        """
        Reset all the data store
        """
        self.buffer = []
        self.history = []

    def add_measure(self, measure):
        """
        Manage buffer of measurements
        """
        self.buffer = self.buffer[1 - self.memorySize:] + [measure or 1]

    def add_delta(self, delta):
        """
        Manage history of deltas
        """
        self.history = self.history[-1:] + [delta]

    def mean_filter(self):
        """
        Take mean value of the self.memorySize latest elements of the buffer
        """
        dist_flt = np.mean(self.buffer[-self.memorySize:])
        if self.consign:
            self.add_delta(self.consign - dist_flt)
        return dist_flt
