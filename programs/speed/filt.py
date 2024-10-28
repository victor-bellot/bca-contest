import numpy as np


class LowPassFilter:

    def __init__(self):
        self.memorySize = 5
        self.meanArea = 3

        self.buffer = []
        self.history = []

        self.farAway = 0.5
        self.consign = None

    def clear_data(self):
        """
        Reset all the data store
        """
        self.buffer = []
        self.history = []

    def set_consign(self, consign):
        """
        Define a consign to follow
        """
        self.consign = consign

    def add_measure(self, measure):
        """
        Manage buffer of measures
        """
        self.buffer = self.buffer[1 - self.memorySize:] + [measure or self.farAway]

    def add_delta(self, delta):
        """
        Manage history of deltas
        """
        self.history = self.history[-1:] + [delta]

    def median_filter(self):
        """
        Take median value of the self.memorySize latest elements of the buffer
        """
        measure_flt = np.median(self.buffer[-self.memorySize:])
        if self.consign:
            self.add_delta(self.consign - measure_flt)
        return measure_flt

    def centered_mean(self):
        """
         If measures = [e1, ..., eN-1]
         sort [eN-self.memorySize, ..., eN-1]
         and return the mean of the self.meanArea elements of its middle
        """
        if len(self.buffer) < self.memorySize:
            return self.median_filter()
        else:  # len(self.buffer) == self.memorySize
            k = self.memorySize // 2 - self.meanArea // 2
            measure_flt = np.mean(np.sort(self.buffer)[k:k + self.meanArea])
            if self.consign:
                self.add_delta(self.consign - measure_flt)
            return measure_flt
