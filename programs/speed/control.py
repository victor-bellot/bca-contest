import time


class RobotControl:

    def __init__(self, flt_front, flt_left, flt_right):
        # set some useful constants
        self.timeStep = 0.2

        self.const0 = 350
        self.const1 = 700 / self.timeStep  # include dt !
        self.adjMax = 100

        self.threshold = 0.75  # line sensor sensitivity
        self.changeTime = 6  # minimum amount of time before changing mode

        self.lateralSonarToWall = 0.235
        self.endDistance = 0.35

        self.turnSpdFollowLine = 25
        self.spdFollowLine = 255

        self.spdFollowWalls = 255

        self.turnSpdSearchLine = 50
        self.spdSearchLine = 255

        self.flt_front = flt_front  # filter for the front sonar sensor
        self.flt_left = flt_left  # filter for the left sonar sensor
        self.flt_right = flt_right  # filter for the right sonar sensor

        consign = self.lateralSonarToWall
        self.flt_left.set_consign(consign)
        self.flt_right.set_consign(consign)

        self.lastDirection = 0  # for line research : -1 to the left / +1 to the right

    def get_measurements(self, rb):
        """
        Take measures with the front, left and right sensor

        input parameters :
        rb : robot object

        output parameters :
        dist_front_flt : filtered front distance
        dist_left_flt : filtered left distance
        dist_right_flt : filtered right distance
        free_left : boolean -> test if dist_left == 0
        free_right : boolean -> test if dist_right == 0
        """

        dist_front, dist_left, dist_right = rb.get_multiple_sonars(['front', 'left', 'right'])

        self.flt_front.add_measure(dist_front)
        dist_front_flt = self.flt_front.median_filter()

        self.flt_left.add_measure(dist_left)
        dist_left_flt = self.flt_left.centered_mean()

        self.flt_right.add_measure(dist_right)
        dist_right_flt = self.flt_right.centered_mean()

        free_left, free_right = (dist_left == 0), (dist_right == 0)

        if not (free_left or free_right):
            self.lastDirection = +1 if dist_left_flt < dist_right_flt else -1

        return dist_front_flt, dist_left_flt, dist_right_flt, free_left, free_right

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

    def follow_line_walled(self, rb):
        """
        Makes rb follow the white line
        until a wall is missing

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdFollowLine
        turn_spd = self.turnSpdFollowLine

        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()

            free_left, free_right = check_free(rb)

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)
            line_detected = max_value > self.threshold

            if free_left or free_right:
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

    def follow_line_turn_left(self, rb):
        """
        Makes rb follow the white line in a left turn
        until there are walls around and no more line

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdFollowLine
        turn_spd = self.turnSpdFollowLine

        self.clear_filters_data()

        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()

            # Prepare for the upcoming follow_walls
            _, _, _, free_left, free_right = self.get_measurements(rb)

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)
            line_detected = max_value > self.threshold

            if not (line_detected or free_left or free_right):
                stage_in_progress = False
            else:
                if left == max_value:
                    delta_spd = 2 * (-turn_spd)
                elif middle == max_value:
                    delta_spd = -turn_spd
                else:
                    delta_spd = 0

                if line_detected:
                    rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def follow_line_turn_right(self, rb):
        """
        Makes rb follow the white line in a right turn
        until there are walls around and no more line

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdFollowLine
        turn_spd = self.turnSpdFollowLine
        init_time = time.time()

        self.clear_filters_data()

        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()

            free_left, free_right = check_free(rb)

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)
            line_detected = max_value > self.threshold

            if not (line_detected or free_left or free_right):
                stage_in_progress = False
            else:
                if (time.time() - init_time < self.changeTime) \
                        or (free_left and free_right):
                    if right == max_value:
                        delta_spd = 2 * turn_spd
                    elif middle == max_value:
                        delta_spd = turn_spd
                    else:
                        delta_spd = 0
                else:
                    delta_spd = 0

                if line_detected:
                    rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def follow_walls(self, rb):
        """
        Makes rg follow the walls around him
        until a wall is missing or a line is detected

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdFollowWalls
        init_time = time.time()

        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()

            _, dist_left_flt, dist_right_flt, free_left, free_right = self.get_measurements(rb)

            if (free_left or free_right or self.detect_line(rb))\
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

    def follow_walls_obstacle(self, rb):
        """
        Makes rg follow the walls around him
        until a wall is missing or an obstacle is encountered

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdFollowWalls
        self.clear_filters_data()

        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()

            dist_front_flt, _, _, _, _ = self.get_measurements(rb)

            if dist_front_flt < self.endDistance:
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
