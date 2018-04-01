import pid
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self, vehicle_mass, wheel_radius, yaw_control_params):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.throttle_control = pid.PID(kp=0.5, ki=0, kd=0, mn=0, mx=1)
        self.yaw_control = YawController(**yaw_control_params)
        self.last_timestamp = None

    def control(self, wp_linear_velocity, wp_angular_velocity,
                current_velocity, dbw_enabled, timestamp):
        # Return throttle, brake, steer
        if dbw_enabled:
            if self.last_timestamp:
                elapsed_time = timestamp-self.last_timestamp
            else:
                elapsed_time = 100000
            self.last_timestamp = timestamp
            vel_diff = wp_linear_velocity-current_velocity
            throttle = self.throttle_control.step(vel_diff, elapsed_time)
            steer = self.yaw_control.get_steering(wp_linear_velocity,
                                                  wp_angular_velocity,
                                                  current_velocity)
            brake = min(-vel_diff, 4)*self.vehicle_mass*self.wheel_radius
            brake = max(brake, 0)
        else:
            self.throttle_control.reset()
            self.last_timestamp = None
            throttle, brake, steer = 0, 0, 0
        return throttle, brake, steer
