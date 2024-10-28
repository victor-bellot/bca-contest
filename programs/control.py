import numpy as np
import time


class RobotControl:

    def __init__(self):
        # set some useful constants
        self.timeStep = 0.2

        self.threshold = 0.75  # line sensor sensitivity

        self.trackWidth = 0.55
        self.centerToWall = 0.25
        self.lateralSonarToWall = 0.235
        self.centerToFront = 0.04
        self.centerToLightning = 0.05

        self.distBetweenWheels = 0.12
        self.wheelDiameter = 0.06
        self.nTicksPerRevol = 512
        self.odoConst = 1

        self.manoeuvreSpd = 20
        self.turnSpd = 100
        self.sampleSpd = 50
        self.searchSpd = 60
        self.turnSpdFollowLine = 40

        self.spdStraightObs = 255
        self.spdStraightCompass = 255
        self.spdFollowLine = 255
        self.spdFollowWalls = 255

        self.compassAngularAccuracy = 1  # in degrees
        self.odoAccuracy = 1  # in ticks

        self.turnStaConst = 2
        self.turnDynConst = 3
        self.straightConst = 1000

        self.magSensorCenter = (0, 0)
        self.magSensorScale = (1, 1)

    def test(self, rb):
        t_init = time.time()
        odo_left_ref, odo_right_ref = rb.get_odometers()

        stage_in_progress = True
        rb.set_speed(255, 255)

        while stage_in_progress:
            t0 = time.time()

            if t0 - t_init > 10:
                stage_in_progress = False
            else:
                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        odo_left, odo_right = rb.get_odometers()
        delta_left, delta_right = odo_left - odo_left_ref, odo_right - odo_right_ref
        d_left = self.wheelDiameter * (np.pi / self.nTicksPerRevol) * delta_left
        d_right = self.wheelDiameter * (np.pi / self.nTicksPerRevol) * delta_right

        return d_left, d_right

    def go_straight_to_obs_compass(self, rb, flt_front):
        """
        Go straight toward rb direction using the compass
        until an obstacle is encountered

        input parameters :
        rb : robot object
        flt_front : filter for the front sonar sensor

        output parameters :
        None
        """

        spd_ask = self.spdStraightCompass
        direction = round_direction(self.get_orientation(rb))

        stage_in_progress = True
        rb.set_speed(spd_ask, spd_ask)

        while stage_in_progress:
            t0 = time.time()

            dist_front = rb.get_sonar('front')
            flt_front.add_measure(dist_front)
            dist_front_flt = flt_front.median_filter()

            dist_obs = dist_front_flt - self.centerToWall

            current_angle = self.get_orientation(rb)
            delta = normalize_angle(direction - current_angle)
            delta_spd = self.turnDynConst * delta

            if 0 < dist_front_flt < self.centerToWall:
                stage_in_progress = False
            else:
                if dist_front_flt == 0:
                    spd = spd_ask
                else:
                    spd = min(spd_ask, self.straightConst * dist_obs)

                rb.set_speed(spd - delta_spd, spd + delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

    def forward_to_line(self, rb, dst, spd):
        """
        Search on a distance dst the white line
        Stop and return True if it find it
        Return False otherwise

        input parameters :
        rb : robot object
        dst : distance to check
        spd : rb speed

        output parameters :
        line_touched : boolean
        """

        n_tick = round((self.nTicksPerRevol * dst) / (np.pi * self.wheelDiameter))
        odo_left_ref, odo_right_ref = rb.get_odometers()

        stage_in_progress = True
        line_touched = False
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()

            odo_left, _ = rb.get_odometers()
            delta_left = n_tick - (odo_left - odo_left_ref)

            _, middle, _ = rb.get_centerline_sensors()

            if middle > self.threshold:
                stage_in_progress = False
                line_touched = True
            elif delta_left < 0:
                stage_in_progress = False
            else:
                rb.set_speed(spd, spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

        return line_touched

    def line_research(self, rb):
        """ SUBJECT TO CHANGE !!!
        Search for the line

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        if max(rb.get_centerline_sensors()) < self.threshold\
                and rb.get_sonar('left') == 0 and rb.get_sonar('right') == 0:

            spd = self.searchSpd
            dst = self.centerToWall / 2

            self.turn_right(rb)
            if self.forward_to_line(rb, dst, spd):
                self.forward_odo(rb, self.centerToLightning, spd)
                self.turn_left(rb)
            else:
                self.turn_back(rb)
                if self.forward_to_line(rb, 2*dst, spd):
                    self.forward_odo(rb, self.centerToLightning, spd)
                self.turn_right(rb)

    def follow_line(self, rb, flt_front):
        """
        Makes rb follow the white line
        until it ends or an obstacle is encountered

        input parameters :
        rb : robot object
        flt_front : filter for the front sonar sensor

        output parameters :
        mission_ended : boolean
        """

        spd = self.spdFollowLine
        turn_spd = self.turnSpdFollowLine

        mission_ended = False
        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress and not mission_ended:
            t0 = time.time()

            dist_front = rb.get_sonar('front')
            flt_front.add_measure(dist_front)
            dist_front_flt = flt_front.median_filter()

            dist_left = rb.get_sonar('left')
            dist_right = rb.get_sonar('right')

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)

            if 0 < dist_front_flt < self.centerToWall:
                mission_ended = True
            elif dist_left and dist_right:
                stage_in_progress = False
            else:
                if left == max_value:
                    delta_spd = -turn_spd
                elif right == max_value:
                    delta_spd = +turn_spd
                else:
                    delta_spd = 0

                if max_value > self.threshold:
                    rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        return mission_ended

    def follow_walls(self, rb, flt_front, flt_left, flt_right):
        """
        Makes rg follow the walls around him
        until an obstacle is encountered

        input parameters :
        rb : robot object
        flt_front : filter for the front sonar sensor
        flt_left : filter for the left sonar sensor
        flt_right : filter for the right sonar sensor

        output parameters :
        mission_ended : boolean
        """

        spd = self.spdFollowWalls
        consign = self.lateralSonarToWall

        flt_left.set_consign(consign)
        flt_right.set_consign(consign)

        mission_ended = False
        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress and not mission_ended:
            t0 = time.time()

            dist_front = rb.get_sonar('front')
            flt_front.add_measure(dist_front)
            dist_front_flt = flt_front.median_filter()

            dist_left = rb.get_sonar('left')
            flt_left.add_measure(dist_left)
            dist_left_flt = flt_left.centered_mean()

            dist_right = rb.get_sonar('right')
            flt_right.add_measure(dist_right)
            dist_right_flt = flt_right.centered_mean()

            left, middle, right = rb.get_centerline_sensors()
            max_value = max(left, middle, right)

            if 0 < dist_front_flt < self.centerToWall:
                mission_ended = True
            elif dist_left_flt == 0 or dist_right_flt == 0 or max_value > self.threshold:
                stage_in_progress = False
            else:
                delta_spd_left = flt_left.get_adjustment()
                delta_spd_right = flt_right.get_adjustment()

                if len(flt_front.buffer) >= flt_front.meanArea:
                    delta_spd = (delta_spd_left - delta_spd_right) / 2
                    rb.set_speed(spd + delta_spd, spd - delta_spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        return mission_ended

    def init_mag_sensor(self, rb):
        """
        Computes magSensorCenter and magSensorScale
        to calibrate the magnetic sensor

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        mag_measures = self.sample_magnetic_field(rb, n_tour=2)
        (xs, ys) = mag_measures['x'], mag_measures['y']

        (x_mean, y_mean) = (np.mean(xs), np.mean(ys))
        (x_max, y_max) = (np.max(xs), np.max(ys))

        self.magSensorCenter = (x_mean, y_mean)
        self.magSensorScale = (x_max - x_mean, y_max - y_mean)

    def sample_magnetic_field(self, rb, n_tour):
        """
        Sample the magnetic field by making rb turning on itself n_tour times

        input parameters :
        rb : robot object
        n_tour : number of tour we make

        output parameters :
        mag_measures : magnetic sensor measures
        """

        mag_measures = {'x': [], 'y': []}

        wheel_angle = (360 * n_tour) * (self.distBetweenWheels / self.wheelDiameter)
        n_tick = round(self.nTicksPerRevol * (wheel_angle / 360))

        odo_left_ref, odo_right_ref = rb.get_odometers()
        spd = self.sampleSpd

        stage_in_progress = True
        rb.set_speed(-spd, spd)

        while stage_in_progress:
            t0 = time.time()

            (x, y) = rb.get_magnetic_sensor()
            mag_measures['x'].append(x)
            mag_measures['y'].append(y)

            odo_left, odo_right = rb.get_odometers()
            delta_left = abs(odo_left - odo_left_ref)
            delta_right = abs(odo_right - odo_right_ref)

            if n_tick <= delta_left and n_tick <= delta_right:
                stage_in_progress = False
            else:
                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

        return mag_measures

    def get_orientation(self, rb):
        """
        Computes the orientation of rb in the magnetic field

        input parameters :
        rb : robot object

        output parameters :
        angle : angle in degree between -180 and +180
        -> 0 : positive x
        -> +90 : positive y
        -> -90 : negative y
        -> +/-180 : negative x
        """

        (mag_x, mag_y) = rb.get_magnetic_sensor()
        (mag_cx, mag_cy) = self.magSensorCenter
        (mag_sx, mag_sy) = self.magSensorScale

        (x, y) = ((mag_x - mag_cx) / mag_sx, (mag_y - mag_cy) / mag_sy)

        return normalize_angle(90 - np.arctan2(y, x) * (180 / np.pi))

    def get_free_direction(self, rb):
        """
        Return the orientation of the free direction
        rg need to follow

        input parameters :
        rb : robot object

        output parameters :
        orientation : angle in degree
        """

        heading = self.get_orientation(rb)
        dist_left = rb.get_sonar('left')
        dist_right = rb.get_sonar('right')

        if dist_left > 0:
            return round_direction(heading - 90)
        elif dist_right > 0:
            return round_direction(heading + 90)
        else:
            return round_direction(heading)

    def turn_to(self, rb, direction):
        """
        Makes rb turn toward a given direction

        input parameters :
        rb : robot object
        direction : the desire direction we want rb to face

        output parameters :
        None
        """

        stage_in_progress = True

        while stage_in_progress:
            t0 = time.time()

            current_angle = self.get_orientation(rb)
            delta = normalize_angle(direction - current_angle)
            abs_delta = abs(delta)

            if abs_delta < self.compassAngularAccuracy:
                stage_in_progress = False
            else:
                axis = round(delta / abs_delta)
                spd = axis * min(self.turnSpd, self.turnStaConst * abs_delta)
                rb.set_speed(-spd, spd)

                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()

    def align(self, rb):
        """
        Align rb to the nearest whole direction

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        direction = self.get_orientation(rb)
        self.turn_to(rb, round_direction(direction))

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

    def go_straight_to_obs(self, rb):
        """
        Solves qualification 1

        input parameters :
        rb : robot object

        output parameters :
        None
        """

        spd = self.spdStraightObs

        stage_in_progress = True
        rb.set_speed(spd, spd)

        while stage_in_progress:
            t0 = time.time()
            dist_front = rb.get_sonar('front')

            if 0 < dist_front < self.centerToWall:
                stage_in_progress = False
            else:
                delta_time = time.time() - t0
                sleep_time = self.timeStep - delta_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        rb.stop()


def normalize_angle(angle):
    """
    Convert angle into its equivalent in the range -180 - +180

    input parameters :
    angle : angle in degree

    output parameters :
    the desired value
    """

    return (angle + 180) % 360 - 180


def round_direction(angle):
    """
    Round an angle to the nearest multiple of 90 degree

    input parameters :
    angle : angle in degree

    output parameters :
    the desired value
    """

    return round(angle / 90) * 90
