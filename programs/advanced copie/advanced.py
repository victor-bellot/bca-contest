import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt  # sensors filtering functions


if __name__ == "__main__":
    pseudo = "CheBello"  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)

    flt_front = filt.LowPassFilter()
    flt_left = filt.LowPassFilter()
    flt_right = filt.LowPassFilter()

    ctrl = control.RobotControl(flt_front, flt_left, flt_right)  # create a robot controller

    while not (ctrl.follow_line(rb) or ctrl.follow_walls(rb)):
        if not ctrl.detect_line(rb):
            print("line research needed")
            ctrl.search_line(rb)

    rb.full_end()
