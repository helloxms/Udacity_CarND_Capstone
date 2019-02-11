from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter8

import rospy



GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, dbw_enabled,
		vehicle_mass, fuel_capacity,
		brake_deadband, decel_limit,
		accel_limit, wheel_radius,
		wheel_base, steer_ratio,
		max_lat_accel, max_steer_angle):
	self.vehicle_mass = vehicle_mass
	self.fuel_capacity = fuel_capacity
	self.brake_deadband = brake_deadband
	self.decel_limit = decel_limit
	self.accel_limit = accel_limit
	self.wheel_radius = wheel_radius

        # TODO: Implement
	min_speed = 0.0
	self.yaw_controller = YawController(wheel_base, steer_ratio,
						min_speed, max_lat_accel,
						max_steer_angle)
	kp = 0.5
	ki = 1.0
	kd = 0.0
	self.pid_throttle = PID(kp, ki, kd, mn=0, mx =accel_limit)
	self.lowpass_a = LowPassFilter8(1., 1., 1., 1., 1., 1., 1., 1.)
	self.lowpass_clv = LowPassFilter8(1., 1., 1., 1., 1., 1., 1., 1.)



    def control(self, dbw_enabled, goal_linear_v, goal_angular_v, brake_control, current_linear_v, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	if not dbw_enabled:
		self.pid_throttle.reset()
		return 0.,0.,0.
	if brake_control > self.brake_deadband:
		current_linear_v = self.lowpass_clv.filt(current_linear_v)
		brake_control = self.lowpass_a.filt(brake_control)
		brake_add_on = self.brake_accel_add_on(current_linear_v)
		brake_control -= brake_add_on
		brake = max(0,brake_control)*self.vehicle_mass*self.wheel_radius
		rospy.loginfo("twist controller brake = %f", brake)
		throttle = 0
	else:
		brake = 0
		error = goal_linear_v - current_linear_v
		throttle = self.pid_throttle.step(error, dt)

	if goal_linear_v < 0.01 and current_linear_v < 0.01:
		steer = 0
	else:
		steer = self.yaw_controller.get_steering(goal_linear_v,
							goal_angular_v,
							current_linear_v)
	return throttle, brake, steer


    def brake_accel_add_on(self, current_linear_v):
	return 0.0995*current_linear_v +0.055
	
