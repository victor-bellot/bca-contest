import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions


if __name__ == "__main__":
    pseudo = "CheBello"  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl()  # create a robot controller

    ctrl.go_straight_to_obs(rb)

    rb.full_end()
