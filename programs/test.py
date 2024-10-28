import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import time


if __name__ == "__main__":
    rb = rob1a.Rob1A()
    ctrl = control.RobotControl()

    print(ctrl.test(rb))

    rb.full_end()
