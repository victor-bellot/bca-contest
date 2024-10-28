import time


class RobotControl:

    def __init__(self, flt_front, flt_left, flt_right):
        # set some useful constants
        self.timeStep = 0.2

        self.const0 = 250
        self.const1 = 500 / self.timeStep  # include dt !
        self.adjMax = 75

        self.threshold = 0.75  # line sensor sensitivity
        self.changeTime = 3  # minimum amount of time before changing mode

        self.centerToWall = 0.2
        self.lateralSonarToWall = 0.235

        self.turnSpdFollowLine = 25
        self.spdFollowLine = 150

        self.spdFollowWalls = 150

        self.turnSpdSearchLine = 20
        self.spdSearchLine = 150

        self.flt_front = flt_front  # filter for the front sonar sensor
        self.flt_left = flt_left  # filter for the left sonar sensor
        self.flt_right = flt_right  # filter for the right sonar sensor

        consign = self.lateralSonarToWall
        self.flt_left.set_consign(consign)
        self.flt_right.set_consign(consign)

        self.lastDirection = 0  # -1 to left / +1 to right

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
        dist_front_flt = self.flt_front.median_filter()

        self.flt_left.add_measure(dist_left)
        dist_left_flt = self.flt_left.centered_mean()

        self.flt_right.add_measure(dist_right)
        dist_right_flt = self.flt_right.centered_mean()

        if dist_left > 0 and dist_right > 0:  # check if we have the walls
            self.lastDirection = +1 if dist_left_flt < dist_right_flt else -1

        return dist_front_flt, dist_left_flt, dist_right_flt

    def detect_line(self, rb):
        """
        Tells if the white line is detected

        input parameters :
        rb : robot object

        output parameters :
        line_detected : boolean
        """

        return max(rb.get_centerline_sensors()) > self.threshold

    def search_line(self, rb):
        """
        Search the white line based on the history of distances to walls

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdSearchLine
        delta_spd = self.lastDirection * self.turnSpdSearchLine

        stage_in_progress = True
        rb.set_speed(spd + delta_spd, spd - delta_spd)

        while stage_in_progress:
            t0 = time.time()

            if self.detect_line(rb):
                stage_in_progress = False
            else:
                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def follow_line(self, rb):
        """
        Makes rb follow the white line
        until there are walls around and no more line

        input parameters :
        rb : robot object

        output parameters :
        mission_ended : boolean
        """

        spd = self.spdFollowLine
        turn_spd = self.turnSpdFollowLine

        mission_ended = False
        stage_in_progress = True
        rb.set_speed(spd, spd)

        while not mission_ended and stage_in_progress:
            t0 = time.time()

            dist_front_flt, _, _ = self.get_measurements(rb)
            free_left, free_right = check_free(rb)

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)
            line_detected = max_value > self.threshold

            if dist_front_flt < self.centerToWall:
                mission_ended = True
            elif not (line_detected or free_left or free_right):
                stage_in_progress = False
            else:
                if left == max_value:
                    delta_spd = -turn_spd
                elif right == max_value:
                    delta_spd = +turn_spd
                else:
                    delta_spd = 0

                if line_detected:
                    rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        return mission_ended

    def follow_walls(self, rb):
        """
        Makes rg follow the walls around him
        until we lost a wall or we find a line

        input parameters :
        rb : robot object

        output parameters :
        mission_ended : boolean
        """

        spd = self.spdFollowWalls
        init_time = time.time()

        mission_ended = False
        stage_in_progress = True
        rb.set_speed(spd, spd)

        while not mission_ended and stage_in_progress:
            t0 = time.time()

            dist_front_flt, _, _ = self.get_measurements(rb)
            free_left, free_right = check_free(rb)

            if dist_front_flt < self.centerToWall:
                mission_ended = True
            elif (self.detect_line(rb) or free_left or free_right)\
                    and (time.time() - init_time > self.changeTime):
                stage_in_progress = False
            else:
                delta_spd_left = self.get_adjustment(self.flt_left)
                delta_spd_right = self.get_adjustment(self.flt_right)

                delta_spd = (delta_spd_left - delta_spd_right) / 2
                rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        return mission_ended

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
