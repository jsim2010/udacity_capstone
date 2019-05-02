from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, decel_limit, vehicle_mass, wheel_radius):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(0.3, 0.1, 0., 0., 0.2)
        self.low_pass_filter = LowPassFilter(0.5, 0.02)
        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()

    def control(self, is_dbw_enabled, linear_velocity, angular_velocity, current_velocity):
        if not is_dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        current_velocity = self.low_pass_filter.filt(current_velocity)
        velocity_error = linear_velocity - current_velocity
        current_time = rospy.get_time()
        throttle = self.throttle_controller.step(velocity_error, current_time - self.last_time)
        brake = 0
        
        if linear_velocity == 0. and current_velocity < 0.1:
            throttle = 0
            brake = 700
        elif throttle < .1 and velocity_error < 0:
            throttle = 0
            brake = abs(max(velocity_error, self.decel_limit)) * self.vehicle_mass * self.wheel_radius
        
        self.last_time = current_time
        
        # Return throttle, brake, steer
        return throttle, 0., self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
