#!/usr/bin/env python3

import numpy as np
import copy
import path_optimizer

import enum
from math import sin, cos, pi, sqrt

import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.insert(0,"/home/aravind/Desktop/Trento/Project_Course/Experiment/Clothoids/lib/lib/")

import G2lib
from G2lib import *
import math

USE_SPIRAL = 1
DISTANCE_BETWEEN_PATH_POINTS = 0.1
Debug = 0
f = open("path.csv", "a")
class LocalPlanner:
    def __init__(self, num_paths, path_offset):
        self._num_paths = num_paths
        self._path_offset = path_offset
        self._path_optimizer = path_optimizer.PathOptimizer()

    def get_goal_state_set(self, goal_index, goal_state, waypoints, ego_state):
        if goal_index < len(waypoints)-1:
            delta_x = waypoints[goal_index+1][0] - waypoints[goal_index][0]
            delta_y = waypoints[goal_index+1][1] - waypoints[goal_index][1]
        else: 
            delta_x = waypoints[goal_index][0] - waypoints[goal_index-1][0]
            delta_y = waypoints[goal_index][1] - waypoints[goal_index-1][1]
        heading = np.arctan2(delta_y,delta_x)

        goal_state_local = copy.copy(goal_state)
        goal_state_local[0] -= ego_state[0] 
        goal_state_local[1] -= ego_state[1] 
        theta = -ego_state[2]
        goal_x = goal_state_local[0] * cos(theta) - goal_state_local[1] * sin(theta)
        goal_y = goal_state_local[0] * sin(theta) + goal_state_local[1] * cos(theta)
        
        goal_t = heading - ego_state[2]
        goal_v = goal_state[2]
        
        if goal_t > pi:
            goal_t -= 2*pi
        elif goal_t < -pi:
            goal_t += 2*pi
        goal_c = goal_state[3]
#        goal_c = 0
        goal_state_set = []
        for i in range(self._num_paths):
            offset = (i - self._num_paths // 2) * self._path_offset

            x_offset = offset * cos(goal_t + pi/2)
            y_offset = offset * sin(goal_t + pi/2)

            goal_state_set.append([goal_x + x_offset, 
                                   goal_y + y_offset, 
                                   goal_t, 
                                   goal_v,
                                   goal_c])
        return goal_state_set

    def plan_paths(self, goal_state_set):
        paths         = []
        path_validity = []
        for goal_state in goal_state_set:

            if USE_SPIRAL:
                path = self._path_optimizer.optimize_spiral(goal_state[0], 
                                                            goal_state[1], 
                                                            goal_state[2])
            else:
                S = G2solve3arc()
                S.build(0.0,0.0,0.0,0.0,goal_state[0],goal_state[1],goal_state[2],0)
                noOfPoints = (int) (S.totalLength()/DISTANCE_BETWEEN_PATH_POINTS) 
                CL = ClothoidList()
                CL.push_back(S.getS0())
                CL.push_back(S.getSM())
                CL.push_back(S.getS1())
                path = [list(),list(),list()]
                path[0].append(S.getS0().xBegin())
                path[1].append(S.getS0().yBegin())
                path[2].append(S.getS0().thetaBegin())
                
                for i in range(noOfPoints-1):
                    if i == 0:
                        continue

                    path_point_CL = CL.evaluate(DISTANCE_BETWEEN_PATH_POINTS*i)                                        
                    path[0].append(path_point_CL[2])
                    path[1].append(path_point_CL[3])
                    path[2].append(path_point_CL[0])

                path[0].append(S.getS1().xEnd())
                path[1].append(S.getS1().yEnd())
                path[2].append(S.getS1().thetaEnd())
                
            if np.linalg.norm([path[0][-1] - goal_state[0], 
                               path[1][-1] - goal_state[1], 
                               path[2][-1] - goal_state[2]]) > 0.1:
                path_validity.append(False)
            else:
                paths.append(path)
                path_validity.append(True)

        if Debug:
            for path in paths:
                for i in range(len(path[0])):
                    f.write(str(path[0][i]) + "," + str(path[1][i]) + "," + str(path[2][i]) + "\n")
                f.write("\n\n")

        if Debug:
            f.write("End of a frame\n")

        return paths, path_validity

    def transform_paths(self,paths, ego_state):
        transformed_paths = []
        for path in paths:
            x_transformed = []
            y_transformed = []
            t_transformed = []

            for i in range(len(path[0])):
                x_transformed.append(ego_state[0] + path[0][i]*cos(ego_state[2]) - \
                                                    path[1][i]*sin(ego_state[2]))
                y_transformed.append(ego_state[1] + path[0][i]*sin(ego_state[2]) + \
                                                    path[1][i]*cos(ego_state[2]))
                t_transformed.append(path[2][i] + ego_state[2])

            transformed_paths.append([x_transformed, y_transformed, t_transformed])

        return transformed_paths
