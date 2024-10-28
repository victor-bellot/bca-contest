import numpy as np


class LowPassFilter:

    def __init__(self):
        self.memorySize = 5
        self.farAway = 0.5

        self.buffer = []
        self.history = []

        self.consign = None

    def clear_data(self):
        """
        Reset all the data stored
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
        self.buffer = self.buffer[1-self.memorySize:] + [measure or self.farAway]

    def add_delta(self, delta):
        """
        Manage history of deltas
        """
        self.history = self.history[-1:] + [delta]

    def no_spike_mean(self):
        """
         Compute the average of the self.memorySize latest measures
         after having removed spikes
        """
        if len(self.buffer) == 1:
            return self.buffer[0]
        else:  # len(self.buffer) == self.memorySize
            sorted_buffer = np.sort(self.buffer)
            if sorted_buffer[-1] > self.farAway:
                measure_flt = np.mean(sorted_buffer[:-1])
            else:
                measure_flt = np.mean(self.buffer)
            if self.consign:
                self.add_delta(self.consign - measure_flt)
            return measure_flt
