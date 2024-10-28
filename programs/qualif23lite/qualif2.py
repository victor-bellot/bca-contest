import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt  # sensors filtering functions


if __name__ == "__main__":
    pseudo = "CheBello"  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl()  # create a robot controller

    flt_front = filt.LowPassFilter()

    ctrl.move_a_bit(rb)
    ctrl.init_mag_sensor(rb)

    for _ in range(2):
        direction = ctrl.get_free_direction(rb)
        ctrl.turn_to(rb, direction)
        ctrl.go_straight_to_obs_compass(rb, flt_front)

    rb.full_end()
