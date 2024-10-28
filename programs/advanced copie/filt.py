import numpy as np


class LowPassFilter:

    def __init__(self):
        self.memorySize = 3
        self.eps = 0.1

        self.buffer = []
        self.history = []

        self.consign = None

    def set_consign(self, consign):
        """
        Define a consign to follow
        """
        self.consign = consign

    def add_measure(self, measure):
        """
        Manage buffer of measures
        """
        self.buffer = self.buffer[1-self.memorySize:] + [measure or 1]

    def add_delta(self, delta):
        """
        Manage history of deltas
        """
        self.history = self.history[-1:] + [delta]

    def no_spike_mean(self):
        """
         If measures = [e1, ..., eN-1]
         sort [eN-self.memorySize, ..., eN-1]
         and return the mean of the self.meanArea elements of its middle
        """
        if len(self.buffer) == 1:
            return self.buffer[0]
        else:  # len(self.buffer) == self.memorySize
            buffer_mean = np.mean(self.buffer)
            sorted_buffer = np.sort(self.buffer)
            if sorted_buffer[-1] - buffer_mean < self.eps:
                measure_flt = buffer_mean
            else:
                measure_flt = np.mean(sorted_buffer[:-1])
            if self.consign:
                self.add_delta(self.consign - measure_flt)
            return measure_flt
