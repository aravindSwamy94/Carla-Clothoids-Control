#!/usr/bin/env python3

import cutils
import numpy as np
from pid import PID
from pure_pursuit import PP


class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        
        ## PIDs
        self.steering_pid        = PID(P=0.34611, I=0.0370736, D=3.5349)      
        self.steering_pid.setSampleTime = 0.01

        self.throttle_brake_pid  = PID(P=7.0, I=1.0, D=1.026185)        
        self.throttle_brake_pid.setSampleTime = 0.01

        self.pp                  = PP(L=4.5, k=1.00, k_Ld=1.3)
        

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
            self.target_wp = self._waypoints[min_idx]
        else:
            desired_speed = self._waypoints[-1][2]
            self.target_wp = self._waypoints[-1]

        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def map_coord_2_Car_coord(self, x, y, yaw, waypoints): 	
	
        wps = np.squeeze(waypoints)
        wps_x = wps[:,0]
        wps_y = wps[:,1]

        num_wp = wps.shape[0]
        
        wp_vehRef = np.zeros(shape=(3, num_wp))
        cos_yaw = np.cos(-yaw)
        sin_yaw = np.sin(-yaw)
                

        wp_vehRef[0,:] = cos_yaw * (wps_x - x) - sin_yaw * (wps_y - y)
        wp_vehRef[1,:] = sin_yaw * (wps_x - x) + cos_yaw * (wps_y - y)        

        return wp_vehRef    

    def update_controls(self):
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        self.vars.create_var('v_previous', 0.0)       

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            wps_vehRef = self.map_coord_2_Car_coord(x, y, yaw, waypoints)
            wps_vehRef_x = wps_vehRef[0,:]
            wps_vehRef_y = wps_vehRef[1,:]

		  

            coeffs = np.polyfit(wps_vehRef_x, wps_vehRef_y, 7)
            CarRef_x = CarRef_y = CarRef_yaw = 0.0

            cte = np.polyval(coeffs, CarRef_x) - CarRef_y
            yaw_err = CarRef_yaw - np.arctan(coeffs[1])

            speed_err = v_desired - v
            
            state = [x, y, yaw, v, cte, yaw_err, speed_err]

            self.throttle_brake_pid.update(speed_err, output_limits = [-1.0, 1.00])            
            if self.throttle_brake_pid.output < 0.0:
                throttle_output = 0    
                brake_output = -self.throttle_brake_pid.output
            else:
                throttle_output = self.throttle_brake_pid.output
                brake_output = 0

            steer_output = self.pp.update(coeffs,v)
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        self.vars.v_previous = v  
