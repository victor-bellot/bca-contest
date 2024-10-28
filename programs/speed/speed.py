import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt  # sensors filtering functions
import time


if __name__ == "__main__":
    t0 = time.time()
    pseudo = "CheBello"  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)

    flt_front = filt.LowPassFilter()
    flt_left = filt.LowPassFilter()
    flt_right = filt.LowPassFilter()

    ctrl = control.RobotControl(flt_front, flt_left, flt_right)  # create a robot controller

    print("Follow line walled", time.time() - t0)
    ctrl.follow_line_walled(rb)

    print("Follow line left", time.time() - t0)
    ctrl.follow_line_turn_left(rb)

    print("Follow walls", time.time() - t0)
    ctrl.follow_walls(rb)

    if not ctrl.detect_line(rb):
        print("Line research needed")
        ctrl.search_line(rb)

    print("Follow line right", time.time() - t0)
    ctrl.follow_line_turn_right(rb)

    print("Follow walls obstacle", time.time() - t0)
    ctrl.follow_walls_obstacle(rb)

    print("Racing time", time.time() - t0)
    rb.full_end()
