import pid
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle_control = pid.PID(kp=0.1, ki=0, kd=0, mn=0, mx=1)
        self.yaw_control = YawController(**kwargs.get('yaw_control_params'))

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
