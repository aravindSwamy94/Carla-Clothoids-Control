#!/usr/bin/env python3


import numpy as np
from math import sin, cos, pi, sqrt

EPSILON = 0.001

class VelocityPlanner:
    def __init__(self, time_gap, a_max, slow_speed, stop_line_buffer):
        self._time_gap         = time_gap
        self._a_max            = a_max
        self._slow_speed       = slow_speed
        self._stop_line_buffer = stop_line_buffer
        self._prev_trajectory  = [[0.0, 0.0, 0.0]]
    def get_open_loop_speed(self, timestep):
        if len(self._prev_trajectory) == 1:
            return self._prev_trajectory[0][2] 
        
        if timestep < 1e-4:
            return self._prev_trajectory[0][2]

        for i in range(len(self._prev_trajectory)-1):
            distance_step = np.linalg.norm(np.subtract(self._prev_trajectory[i+1][0:2], 
                                                       self._prev_trajectory[i][0:2]))
            velocity = self._prev_trajectory[i][2]
            time_delta = distance_step / velocity
            if time_delta > timestep:
                v1 = self._prev_trajectory[i][2]
                v2 = self._prev_trajectory[i+1][2]
                v_delta = v2 - v1
                interpolation_ratio = timestep / time_delta
                return v1 + interpolation_ratio * v_delta

            else:
                timestep -= time_delta

        return self._prev_trajectory[-1][2]

    def compute_velocity_profile(self, path, desired_speed, ego_state, 
                                 closed_loop_speed, decelerate_to_stop):
        profile = []
        # For our profile, use the open loop speed as our initial speed.
        start_speed = ego_state[3]
        # Generate a trapezoidal profile to decelerate to stop.
        if decelerate_to_stop:
            profile = self.decelerate_profile(path, start_speed)

        else:
            profile = self.nominal_profile(path, start_speed, desired_speed)

        if len(profile) > 1:
            interpolated_state = [(profile[1][0] - profile[0][0]) * 0.1 + profile[0][0], 
                                  (profile[1][1] - profile[0][1]) * 0.1 + profile[0][1], 
                                  (profile[1][2] - profile[0][2]) * 0.1 + profile[0][2]]
            del profile[0]
            profile.insert(0, interpolated_state)

        self._prev_trajectory = profile

        return profile

    def decelerate_profile(self, path, start_speed): 
        profile          = []
        slow_speed       = self._slow_speed
        stop_line_buffer = self._stop_line_buffer

        decel_distance = calc_distance(start_speed, slow_speed, -self._a_max)
        brake_distance = calc_distance(slow_speed, 0, -self._a_max)

        path_length = 0.0
        for i in range(len(path[0])-1):
            path_length += np.linalg.norm([path[0][i+1] - path[0][i], 
                                           path[1][i+1] - path[1][i]])

        stop_index = len(path[0]) - 1
        temp_dist = 0.0
        while (stop_index > 0) and (temp_dist < stop_line_buffer):
            temp_dist += np.linalg.norm([path[0][stop_index] - path[0][stop_index-1], 
                                         path[1][stop_index] - path[1][stop_index-1]])
            stop_index -= 1

        if brake_distance + decel_distance + stop_line_buffer > path_length:
            speeds = []
            vf = 0.0
            for i in reversed(range(stop_index, len(path[0]))):
                speeds.insert(0, 0.0)
            for i in reversed(range(stop_index)):
                dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                       path[1][i+1] - path[1][i]])
                vi = calc_final_speed(vf, -self._a_max, dist)
                if vi > start_speed:
                    vi = start_speed

                speeds.insert(0, vi)
                vf = vi

            for i in range(len(speeds)):
                profile.append([path[0][i], path[1][i], speeds[i]])
            
        else:
            brake_index = stop_index 
            temp_dist = 0.0

            while (brake_index > 0) and (temp_dist < brake_distance):
                temp_dist += np.linalg.norm([path[0][brake_index] - path[0][brake_index-1], 
                                             path[1][brake_index] - path[1][brake_index-1]])
                brake_index -= 1

            decel_index = 0
            temp_dist = 0.0
            while (decel_index < brake_index) and (temp_dist < decel_distance):
                temp_dist += np.linalg.norm([path[0][decel_index+1] - path[0][decel_index], 
                                             path[1][decel_index+1] - path[1][decel_index]])
                decel_index += 1

            # The speeds from the start to decel_index should be a linear ramp
            # from the current speed down to the slow_speed, decelerating at
            # -self._a_max.
            vi = start_speed
            for i in range(decel_index): 
                dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                       path[1][i+1] - path[1][i]])
                vf = calc_final_speed(vi, -self._a_max, dist)
                # We don't want to overshoot our slow_speed, so clamp it to that.
                if vf < slow_speed:
                    vf = slow_speed

                profile.append([path[0][i], path[1][i], vi])
                vi = vf

            # In this portion of the profile, we are maintaining our slow_speed.
            for i in range(decel_index, brake_index):
                profile.append([path[0][i], path[1][i], vi])
                
            # The speeds from the brake_index to stop_index should be a
            # linear ramp from the slow_speed down to the 0, decelerating at
            # -self._a_max.
            for i in range(brake_index, stop_index):
                dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                       path[1][i+1] - path[1][i]])
                vf = calc_final_speed(vi, -self._a_max, dist)
                profile.append([path[0][i], path[1][i], vi])
                vi = vf

            # The rest of the profile consists of our stop_line_buffer, so
            # it contains zero speed for all points.
            for i in range(stop_index, len(path[0])):
                profile.append([path[0][i], path[1][i], 0.0])

        return profile

    def nominal_profile(self, path, start_speed, desired_speed):
        profile = []
        # Compute distance travelled from start speed to desired speed using
        # a constant acceleration.
        if desired_speed < start_speed:
            accel_distance = calc_distance(start_speed, desired_speed, -self._a_max)
        else:
            accel_distance = calc_distance(start_speed, desired_speed, self._a_max)

        # Here we will compute the end of the ramp for our velocity profile.
        # At the end of the ramp, we will maintain our final speed.
        ramp_end_index = 0
        distance = 0.0
        while (ramp_end_index < len(path[0])-1) and (distance < accel_distance):
            distance += np.linalg.norm([path[0][ramp_end_index+1] - path[0][ramp_end_index], 
                                        path[1][ramp_end_index+1] - path[1][ramp_end_index]])
            ramp_end_index += 1

        # Here we will actually compute the velocities along the ramp.
        vi = start_speed
        for i in range(ramp_end_index):
            dist = np.linalg.norm([path[0][i+1] - path[0][i], 
                                   path[1][i+1] - path[1][i]])
            if desired_speed < start_speed:
                vf = calc_final_speed(vi, -self._a_max, dist)
                # clamp speed to desired speed
                if vf < desired_speed:
                    vf = desired_speed
            else:
                vf = calc_final_speed(vi, self._a_max, dist)
                # clamp speed to desired speed
                if vf > desired_speed:
                    vf = desired_speed

            profile.append([path[0][i], path[1][i], vi])
            vi = vf

        for i in range(ramp_end_index+1, len(path[0])):
            profile.append([path[0][i], path[1][i], desired_speed])

        return profile

def calc_distance(v_i, v_f, a):
    if np.abs(a) < EPSILON:
        d = np.inf
    else:
        d = (v_f**2 - v_i**2) / (2 * a)
    return d

def calc_final_speed(v_i, a, d):
    pass

    temp = v_i*v_i+2*d*a
    if temp < 0: return 0.0000001
    else: return sqrt(temp)

