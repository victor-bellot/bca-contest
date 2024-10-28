import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt  # sensors filtering functions


if __name__ == "__main__":
    pseudo = "CheBello"  # you can define your pseudo here
    rb = rob1a.Rob1A()  # create a robot (instance of Rob1A class)

    flt_front = filt.LowPassFilter()
    flt_left = filt.LowPassFilter()
    flt_right = filt.LowPassFilter()

    ctrl = control.RobotControl(flt_front, flt_left, flt_right)  # create a robot controller

    ctrl.move_a_bit(rb)
    for _ in range(2):
        ctrl.align_to_free_direction(rb)
        ctrl.follow_path_to_obstacle(rb)

    rb.full_end()
