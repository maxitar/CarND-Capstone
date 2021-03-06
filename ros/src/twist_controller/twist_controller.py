import pid
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # def __init__(self, *args, **kwargs):
    def __init__(self, vehicle_mass, wheel_radius, decel_limit,
                 accel_limit, yaw_control_params):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.throttle_control = pid.PID(kp=0.5, ki=0, kd=0, mn=0,
                                        mx=accel_limit)
        self.yaw_control = YawController(**yaw_control_params)
        self.lpf_velocity = LowPassFilter(tau=0.5, ts=0.2)
        self.last_timestamp = None
        self.last_throttle = 0

    def control(self, wp_linear_velocity, wp_angular_velocity,
                current_velocity, dbw_enabled, timestamp):
        # Return throttle, brake, steer
        if dbw_enabled:
            elapsed_time = 1 # Does not matter since ki=kd=0
            self.last_timestamp = timestamp
            original_velocity = current_velocity
            current_velocity = self.lpf_velocity.filt(current_velocity)
            vel_diff = wp_linear_velocity-current_velocity
            throttle = self.throttle_control.step(vel_diff, elapsed_time)
            # Limit throttle growth, but let it decrease as usual
            throttle = min(self.last_throttle+0.002, throttle)
            self.last_throttle = throttle
            steer = self.yaw_control.get_steering(wp_linear_velocity,
                                                  wp_angular_velocity,
                                                  current_velocity)
            if current_velocity < 1. and wp_linear_velocity <= 0.01:
                throttle = 0
                brake = 400
            else:
                brake = min(-vel_diff, abs(self.decel_limit))*self.vehicle_mass*self.wheel_radius
                brake = max(brake, 0)
        else:
            self.throttle_control.reset()
            self.last_timestamp = None
            throttle, brake, steer = 0, 0, 0
        return throttle, brake, steer
