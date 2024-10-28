import time


class RobotControl:

    def __init__(self):
        # set some useful constants
        self.timeStep = 0.2
        self.centerToWall = 0.25
        self.spdStraightObs = 255

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
