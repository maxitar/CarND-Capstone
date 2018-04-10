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
        # self.throttle_control = pid.PID(kp=0.3, ki=0.1, kd=0, mn=0, mx=0.2)
        self.throttle_control = pid.PID(kp=0.5, ki=0, kd=0, mn=0, mx=0.5)
        self.yaw_control = YawController(**yaw_control_params)
        self.lpf_velocity = LowPassFilter(tau=0.5, ts=0.02)
        self.last_timestamp = None

    def control(self, wp_linear_velocity, wp_angular_velocity,
                current_velocity, dbw_enabled, timestamp):
        # Return throttle, brake, steer
        if dbw_enabled:
            if self.last_timestamp:
                elapsed_time = timestamp-self.last_timestamp
            else:
                elapsed_time = 0.00000001 # 100000
            self.last_timestamp = timestamp
            current_velocity = self.lpf_velocity.filt(current_velocity)
            vel_diff = wp_linear_velocity-current_velocity
            throttle = self.throttle_control.step(vel_diff, elapsed_time)
            steer = self.yaw_control.get_steering(wp_linear_velocity,
                                                  wp_angular_velocity,
                                                  current_velocity)
            if current_velocity < 0.1 and wp_linear_velocity <= 0.01:
                brake = 400
            else:
                brake = min(-vel_diff, abs(self.decel_limit))*self.vehicle_mass*self.wheel_radius
                brake = max(brake, 0)
        else:
            self.throttle_control.reset()
            self.last_timestamp = None
            throttle, brake, steer = 0, 0, 0
        return throttle, brake, steer
