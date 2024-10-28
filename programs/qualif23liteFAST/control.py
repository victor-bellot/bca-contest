import numpy as np
import time


class RobotControl:

    def __init__(self, flt_front, flt_left, flt_right):
        # set some useful constants
        self.timeStep = 0.2

        self.threshold = 0.75  # line sensor sensitivity
        self.adjMax = 50

        self.const0 = 350
        self.const1 = 700 / self.timeStep  # include dt

        self.lateralSonarToWall = 0.235
        self.centerToWall = 0.25
        self.centerToFront = 0.04

        self.distBetweenWheels = 0.12
        self.wheelDiameter = 0.06
        self.nTicksPerRevol = 512

        self.manoeuvreSpd = 20
        self.turnSpd = 100
        self.followSpd = 255
        self.followLineTurnSpd = 15

        self.odoConst = 1
        self.odoAccuracy = 3  # in ticks
        self.obsConst = 1000
        self.obsAccuracy = 0.001

        self.flt_front = flt_front  # filter for the front sonar sensor
        self.flt_left = flt_left  # filter for the left sonar sensor
        self.flt_right = flt_right  # filter for the right sonar sensor

        consign = self.lateralSonarToWall
        self.flt_left.set_consign(consign)
        self.flt_right.set_consign(consign)

    def get_measurements(self, rb):
        """
        Take measures with the front, left and right sensor

        input parameters :
        rb : robot object

        output parameters :
        dist_front_flt : filtered front distance
        dist_left_flt : filtered left distance
        dist_right_flt : filtered right distance
        """

        dist_front, dist_left, dist_right = rb.get_multiple_sonars(['front', 'left', 'right'])

        self.flt_front.add_measure(dist_front)
        dist_front_flt = self.flt_front.mean_filter()

        self.flt_left.add_measure(dist_left)
        dist_left_flt = self.flt_left.mean_filter()

        self.flt_right.add_measure(dist_right)
        dist_right_flt = self.flt_right.mean_filter()

        return dist_front_flt, dist_left_flt, dist_right_flt

    def follow_path_to_obstacle(self, rb):
        """
        A mix of follow_line and follow_walls with ending when an obstacle is encountered
        follow_line is preferred if a line is detected
        otherwise follow_walls is used with the available walls

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd_ask = self.followSpd
        turn_spd = self.followLineTurnSpd

        self.clear_filters_data()

        stage_in_progress = True
        rb.set_speed(spd_ask, spd_ask)

        while stage_in_progress:
            t0 = time.time()

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)
            line_detected = max_value > self.threshold

            dist_front_flt, dist_left_flt, dist_right_flt = self.get_measurements(rb)
            delta_obs = dist_front_flt - self.centerToWall

            free_left, free_right = check_free(rb)

            if abs(delta_obs) < self.obsAccuracy:
                stage_in_progress = False
            else:
                if line_detected:
                    if left == max_value:
                        delta_spd = -turn_spd
                    elif right == max_value:
                        delta_spd = +turn_spd
                    else:
                        delta_spd = 0
                elif not (free_left and free_right) and delta_obs > 0.05:
                    delta_spd_left = self.get_adjustment(self.flt_left)
                    delta_spd_right = self.get_adjustment(self.flt_right)

                    if free_left:
                        delta_spd = -delta_spd_right
                    elif free_right:
                        delta_spd = delta_spd_left
                    else:
                        delta_spd = (delta_spd_left - delta_spd_right) / 2
                else:
                    delta_spd = 0

                if dist_front_flt == 0:
                    spd = spd_ask
                else:
                    if delta_obs > 0:
                        spd = min(spd_ask, self.obsConst * delta_obs)
                    else:
                        spd = max(-spd_ask, self.obsConst * delta_obs)

                rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

    def forward_odo(self, rb, dst, max_spd):
        """
        Makes rb moves of dst forward using the odometer

        input parameters :
        rb : robot object
        dst : the relative distance we want to move of (in meters)
        max_spd : maximum speed the robot travels

        output parameters :
        None
        """

        eps = self.odoAccuracy
        n_tick = round((self.nTicksPerRevol * dst) / (np.pi * self.wheelDiameter))
        odo_left_ref, odo_right_ref = rb.get_odometers()

        stage_in_progress = True

        while stage_in_progress:
            t0 = time.time()

            odo_left, odo_right = rb.get_odometers()
            delta_left = n_tick - (odo_left - odo_left_ref)
            delta_right = n_tick - (odo_right - odo_right_ref)

            if abs(delta_left) <= eps and abs(delta_right) <= eps:
                stage_in_progress = False
            else:
                spd_left = min(self.odoConst * delta_left, max_spd)
                spd_right = min(self.odoConst * delta_right, max_spd)
                rb.set_speed(spd_left, spd_right)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

    def move_a_bit(self, rb):
        """
        Adjust rb position on the circuit

        input parameters :
        rb : robot object
        sense : +/-1 forward or backward

        output parameters :
        None
        """

        self.forward_odo(rb, dst=-self.centerToFront, max_spd=self.manoeuvreSpd)

    def turn_odo(self, rb, angle):
        """
        Turn rb of angle using the odometer

        input parameters :
        rb : robot object
        angle : angle of rotation

        output parameters :
        None
        """

        wheel_angle = angle * (self.distBetweenWheels / self.wheelDiameter)
        n_tick = round(self.nTicksPerRevol * (wheel_angle / 360))

        _, odo_right_ref = rb.get_odometers()
        spd = self.turnSpd

        stage_in_progress = True

        while stage_in_progress:
            t0 = time.time()

            _, odo_right = rb.get_odometers()
            delta_right = n_tick - (odo_right - odo_right_ref)
            abs_delta_right = abs(delta_right)

            if abs_delta_right <= self.odoAccuracy:
                stage_in_progress = False
            else:
                axis = round(delta_right / abs_delta_right)
                spd_right = axis * min(self.odoConst * abs_delta_right, spd)
                rb.set_speed(-spd_right, spd_right)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

    def turn_left(self, rb):
        self.turn_odo(rb, +90)

    def turn_right(self, rb):
        self.turn_odo(rb, -90)

    def turn_back(self, rb):
        self.turn_odo(rb, 180)

    def align_to_free_direction(self, rb):
        """
        Return the orientation of the free direction
        rg need to follow

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        free_left, free_right = check_free(rb)

        if free_left:
            self.turn_left(rb)
        elif free_right:
            self.turn_right(rb)

    def get_adjustment(self, flt):
        """
        Return adjustment needed to go to consign

        input parameters :
        flt : filter

        output parameters :
        adj : adjustment
        """

        adj = 0
        deltas = flt.history

        if len(deltas) == 1:
            adj = deltas[-1] * self.const0
        elif len(deltas) > 1:
            adj = deltas[-1] * self.const0 \
                  + (deltas[-1] - deltas[-2]) * self.const1

        abs_adj = abs(adj)
        if abs_adj > 0:
            return (adj / abs_adj) * min(self.adjMax, abs_adj)
        else:
            return adj

    def clear_filters_data(self):
        """
        Clear filters data
        """
        self.flt_front.clear_data()
        self.flt_left.clear_data()
        self.flt_right.clear_data()


def check_free(rb):
    """
    Tells if the left or the right direction is free

    input parameters :
    rb : robot object

    output parameters :
    free_left : boolean -> test if dist_left == 0
    free_right : boolean -> test if dist_right == 0
    """

    dist_left, dist_right = rb.get_multiple_sonars(['left', 'right'])
    free_left, free_right = (dist_left == 0), (dist_right == 0)

    return free_left, free_right
