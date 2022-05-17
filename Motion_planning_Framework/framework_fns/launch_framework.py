from __future__ import print_function


import sys
import os
import argparse
import logging
import time
import math
import numpy as np
import csv
import matplotlib.pyplot as plt
import controller2d
import configparser 
import local_planner
#import behavioural_planner
import collision_checker

import collections
import datetime
import glob
import random
import re
import weakref

import live_plotter as lv

import stop_planner_fns as stop_planner
import common_planner_fns as common_planner
import velocity_planner
import behaviour_planner as bp
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/../carla')
    #sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
from agents.navigation.roaming_agent import *
from agents.navigation.basic_agent import *
from agents.navigation.global_route_planner import GlobalRoutePlanner
#from agents.navigation.global_route_planner import NavEnum
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from shapely.geometry import Point, Polygon


"""
Simulation params
"""

TOTAL_RUN_TIME         = 100000.00 # game seconds (total runtime before sim end)

DIST_THRESHOLD_TO_LAST_WAYPOINT = 4  # some distance from last position before
                                       # simulation ends

ENABLE_PLOT = 1
# Planning Constants
vehicle_speed          =  7                # m/s
NUM_PATHS              =  6
BP_LOOKAHEAD_BASE      = 16.0             # m
BP_LOOKAHEAD_TIME      = 1.0              # s
PATH_OFFSET            = 1.5              # m
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.5, 1.5, 1.5]  # m
TIME_GAP               = 2.0              # s
PATH_SELECT_WEIGHT     = 10
A_MAX                  = 1.5              # m/s^2
SLOW_SPEED             = 0.0              # m/s
STOP_LINE_BUFFER       = 0.0              # m

LP_FREQUENCY_DIVISOR   = 2                # Frequency divisor to make the 
                                          # local planner operate at a lower
                                          # frequency than the controller


FIGSIZE_X_INCHES   = 16      # x figure size of feedback in inches
FIGSIZE_Y_INCHES   = 16      # y figure size of feedback in inches
PLOT_LEFT          = 0.1    # in fractions of figure width and height
PLOT_BOT           = 0.1    
PLOT_WIDTH         = 0.8
PLOT_HEIGHT        = 0.8

STOP_SIGN_FENCELENGTH = 5        # m


INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # selected path

INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points


def check_for_stop(collision_check_array,paths,expanded_car_path):
    if expanded_car_path is not None:
        poly = Polygon(expanded_car_path)
        for index,item in enumerate(collision_check_array):
            if item == True:
                path = paths[index]
                for i in range(len(path[0])):
                    point = Point(path[0][i],path[1][i])
                    if point.within(poly):
                        return True

        return False

def expandActorTrajectories(Path,Map):
    expandedPath = []
    for item in Path:
        waypoint = Map.get_waypoint(carla.Location(item[0],item[1],0))
        newPoint = Map.get_waypoint(carla.Location(item[0],item[1],0))

        newPoint_x = waypoint.transform.location.x + ((waypoint.lane_width/2)* (math.cos(math.radians(iConvertTo360(waypoint.transform.rotation.yaw)+ 90 ))))
        newPoint_y = waypoint.transform.location.y + ((waypoint.lane_width/2) * (math.sin(math.radians(iConvertTo360(waypoint.transform.rotation.yaw)+ 90 ))))
        expandedPath.append((newPoint_x,newPoint_y))

    for item in reversed(Path):
        waypoint = Map.get_waypoint(carla.Location(item[0],item[1],0))
        newPoint = waypoint
        newPoint_x = waypoint.transform.location.x + ((waypoint.lane_width/2)* (math.cos(math.radians(iConvertTo360(waypoint.transform.rotation.yaw)- 90 ))))
        newPoint_y = waypoint.transform.location.y + ((waypoint.lane_width/2) * (math.sin(math.radians(iConvertTo360(waypoint.transform.rotation.yaw)- 90 ))))
        expandedPath.append((newPoint_x,newPoint_y))

    expandedPath.append((expandedPath[0][0],expandedPath[0][1]))

    return expandedPath

def getActorTrajectories(Actor,Map,World):
    if Actor is not None:
        prediction_period = 10
        velocityOfActor= get_actor_velocity(Actor)
#        print(velocityOfActor)
        if velocityOfActor < 1:
             distance_required=10
             path_points_distance = 1
        else: 
            distance_required = velocityOfActor*prediction_period
            path_points_distance = 3
        behind_distance = Actor.bounding_box.extent.x * 4
        Actor.set_simulate_physics(True)
        waypoint = Map.get_waypoint(Actor.get_location())
        behind_point_x = Actor.get_location().x + ((behind_distance)* (math.cos(math.radians(iConvertTo360(Actor.get_transform().rotation.yaw)+ 180 ))))
        behind_point_y = Actor.get_location().y + ((behind_distance)* (math.sin(math.radians(iConvertTo360(Actor.get_transform().rotation.yaw)+ 180 ))))
        pathList=[]
        pathList.append([behind_point_x,behind_point_y])
        pathPoint = Map.get_waypoint(Actor.get_location())
        pathList.append([Actor.get_transform().location.x,Actor.get_transform().location.y])
        for j in range(int(distance_required/path_points_distance)):
            nextWayPoint= pathPoint.next(path_points_distance)
            pathPoint=nextWayPoint[0]
            pathList.append([nextWayPoint[0].transform.location.x,nextWayPoint[0].transform.location.y])
        return pathList


def check_for_valid_goal(goal_set,map,waypoints):
    new_goal_set = []
    for item in goal_set:
        new_wp = map.get_waypoint(carla.Location(item[0],item[1],0),False,lane_type=carla.LaneType.Driving)
        if new_wp is not None:
            new_goal_set.append(item)
    return new_goal_set

# ==============================================================================
#-- iConvertTo360()-------------------------------------------------------

# Obtains an angle in [-180,180] format and converts it to [0,360] format

#    Args:
#          x : angle in [-180,180] format,degrees
#    Returns:
#          x: angle in [0,360] format, degrees  

def iConvertTo360(x):
    if x >0:
        x+=0.0
    else:
        x=360+x
    return x



# ==============================================================================
#-- get_current_pose()-------------------------------------------------------

#Obtains current x,y,yaw pose from the client measurements
    
#    Obtains the current x,y, and yaw pose from the client measurements.

#   Args:
#        ego: The ego vehicle (carla.Actor)
#
#    Returns: (x, y, yaw)
#        x: X position in meters
#        y: Y position in meters
#        yaw: Yaw position in radians

# ==============================================================================


def get_current_pose(ego):
    """    """
    x   = ego.get_transform().location.x
    y   = ego.get_transform().location.y
    yaw = math.radians(ego.get_transform().rotation.yaw)

    return (x, y, yaw)

# ==============================================================================
#-- get_actor_velocity()-------------------------------------------------------

#Obtains ego information
    

#   Args:
#        ego: The ego vehicle (carla.Actor)
#
#    Returns: resultant velocity in m/s


# ==============================================================================

def get_actor_velocity(Actor):
    velX = Actor.get_velocity().x * Actor.get_velocity().x
    velY = Actor.get_velocity().y * Actor.get_velocity().y
    velZ = Actor.get_velocity().z * Actor.get_velocity().z
    return math.sqrt(velX+velY+velZ)


# ==============================================================================
#-- check_actor_static()-------------------------------------------------------

#Obtains ego information
    

#   Args:
#        ego: The ego vehicle (velocity)
#
#    Returns: bool. True if actor is static or False it is dynamic


# ==============================================================================

def check_actor_static(velocity):
    if velocity < 1:
        return True
    else:
        return False



# ==============================================================================
#-- send_control_command()-------------------------------------------------------

#    Send control command to CARLA client.
#    
#    Send control command to CARLA client.
#
#    Args:
#        client: The CARLA client object
#       throttle: Throttle command for the sim car [0, 1]
#        steer: Steer command for the sim car [-1, 1]
#        brake: Brake command for the sim car [0, 1]
#        hand_brake: Whether the hand brake is engaged
#        reverse: Whether the sim car is in the reverse gear
#   
# ==============================================================================



def send_control_command(ego, throttle, steer, brake, 
                         hand_brake=False, reverse=False):
    control = carla.VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    ego.apply_control(control)

# =============================================================================
#-- getCurvature() ------------------------------------------------------------
# =============================================================================

#Below function is modified based three point curvature formula
#http://m.intmath.com/applications-differentiation/8-radius-curvature.php (Using Linear Approximations and Calculus Methods)

def getCurvature(waypoints):

    if len(waypoints)< 5:
        for i in range(len(waypoints)):
            waypoints[i].append(0)
        return waypoints

    for i in range(len(waypoints)):
        pos = waypoints[i]
        maxStep = 5
        maxStep = min(min(5,i),len(waypoints)-i -1)
        if maxStep ==0:
            waypoints[i].append(0)
            continue
        iminus = i-maxStep
        iplus = i+ maxStep
        if iminus < 0 :
            iminus = iminus + len(waypoints)
        pminus = waypoints[iminus]

        if iplus > len(waypoints):
           iplus = iplus -len(waypoints)
       
        pplus = waypoints[iplus]

        dx = (pplus[1] - pminus[1]) / (pplus[0] - pminus[0])
        ddx = ((pplus[1] - pos[1]) - (pos[1] - pminus[1])) / (pplus[0] - pminus[0])
        if ddx == 0.0:
            curvature =0
        else:    
            curvature = 1 /( pow(math.sqrt(1 + pow(dx,2)),3) / (ddx));
        waypoints[i].append(curvature)
    return waypoints
        


# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    #decision = sys.argv[1]
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'hero':
            ego = actor
            break

#    world.set_weather(carla.WeatherParameters(cloudiness=15.000000, precipitation=0.000000, precipitation_deposits=0.000000, wind_intensity=0.350000, sun_azimuth_angle=0.000000, sun_altitude_angle=75.000000))

    
    # Coordinates for Town03
#    ego.set_transform(carla.Transform(carla.Location(x=233.35, y=81.805, z=0.0472859), carla.Rotation(pitch=0.700053, yaw=91.977, roll=0.0388931)))# Setting the ego vehicle to a set source psition
#    destination = carla.Transform(carla.Location(x=-78.2409, y=150.354, z=-0.0135565), carla.Rotation(pitch=-0.153536, yaw=-89.9945, roll=-0.000701898)) # Setting the destination (Right now hard coded, Can be integrated with some config file later)
    

    # Coordinates for Town01
    ego.set_transform(carla.Transform(carla.Location(x=396.251892, y=302.294495, z=-0.004815), carla.Rotation(pitch=0.093000, yaw=-89.927544, roll=0.000347)))# Setting the ego vehicle to a set source psition
    destination = carla.Transform(carla.Location(x=396.181946, y=116.861641, z=-0.004879), carla.Rotation(pitch=0.096060, yaw=-89.715233, roll=0.000016)) # Setting the destination (Right now hard coded, Can be integrated with some config file later)

    DAO_GRP = GlobalRoutePlannerDAO(map,2.0)# Using the GRP of the carla module. Which gives output of 1m waypoints 
    GRP =  GlobalRoutePlanner(DAO_GRP)
    GRP.setup()
    grp_wps = GRP.trace_route(ego.get_transform().location,destination.location)

    
    waypoints = []
    waypoints_iter=0
    for wp in grp_wps:
        if waypoints_iter%2==0:
            waypoints.append([wp[0].transform.location.x,wp[0].transform.location.y,vehicle_speed])
        waypoints_iter = waypoints_iter+1    

    waypoints = getCurvature(waypoints)
    controller = controller2d.Controller2D(waypoints)


    TOTAL_EPISODE_FRAMES = int(TOTAL_RUN_TIME)


    start_timestamp = world.get_snapshot().timestamp.elapsed_seconds    
    start_x, start_y, start_yaw = get_current_pose(ego)
    send_control_command(ego, throttle=0.0, steer=0, brake=1.0)
    x_history     = [start_x]
    y_history     = [start_y]
    yaw_history   = [start_yaw]
    time_history  = [0]
    speed_history = [0]
    collided_flag_history = [False]  


    if ENABLE_PLOT:
        lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")

    if ENABLE_PLOT:
        trajectory_fig = lp_traj.plot_new_dynamic_2d_figure(
                title='Vehicle Trajectory',
                figsize=(FIGSIZE_X_INCHES, FIGSIZE_Y_INCHES),
                edgecolor="black",
                rect=[PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT])

    if ENABLE_PLOT:
        trajectory_fig.set_invert_x_axis() # Because UE4 uses left-handed 
                                           # coordinate system the X
                                           # axis in the graph is flipped
    if ENABLE_PLOT:
        trajectory_fig.set_axis_equal()    # X-Y spacing should be equal in size



    waypoints_np = np.array(waypoints)
    if ENABLE_PLOT:
#        trajectory_fig.add_graph("waypoints", window_size=waypoints_np.shape[0],
#                             x0=waypoints_np[:,0], y0=waypoints_np[:,1],
#                             linestyle="-", marker="", color='g')
        trajectory_fig.add_graph("selected_path", 
                                 window_size=INTERP_MAX_POINTS_PLOT,
                                 x0=[start_x]*INTERP_MAX_POINTS_PLOT, 
                                 y0=[start_y]*INTERP_MAX_POINTS_PLOT,
                                 color=[1, 0.5, 0.0],
                                 linewidth=3)
        for i in range(NUM_PATHS):
            trajectory_fig.add_graph("local_path " + str(i), window_size=200,
                                 x0=None, y0=None, color=[0.0, 0.0, 1.0])




    landmarks = []
    stopsign_data = []
    stopsign_fences = []     # [x0, y0, x1, y1]

    parkedcar_data=[]
    parkedcar_fence = []
    parkedcar_box_pts = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
      # [x,y]
    parked_car_line_fences =[]

    

    parkedcar_box_pts_np = np.array(parkedcar_box_pts)
    if ENABLE_PLOT:    
        trajectory_fig.add_graph("parkedcar_pts", window_size=waypoints_np.shape[0],
                             x0=ego.get_transform().location.x, y0=ego.get_transform().location.y,
                             linestyle="", marker="+", color='b')
        trajectory_fig.add_graph("dynamiccar_path", 
                                 window_size=INTERP_MAX_POINTS_PLOT,
                                 x0=[start_x]*INTERP_MAX_POINTS_PLOT, 
                                 y0=[start_y]*INTERP_MAX_POINTS_PLOT,
                                 color=[1, 1.0, 0.0],
                                 linewidth=4)

        



    local_waypoints = None
    path_validity   = np.zeros((NUM_PATHS, 1), dtype=bool)
    lp = local_planner.LocalPlanner(NUM_PATHS,
                                    PATH_OFFSET)
                                    
                                    
    vp = velocity_planner.VelocityPlanner(TIME_GAP,A_MAX,SLOW_SPEED,STOP_LINE_BUFFER)


    cc = collision_checker.CollisionChecker(CIRCLE_OFFSETS,CIRCLE_RADII,PATH_SELECT_WEIGHT)


    for frame in range(TOTAL_EPISODE_FRAMES):       
        try:

            dynamic_obstacle_data = []

            parkedcar_box_pts = []
    
            landmarks = []
            stopsign_data = []


            parkedcar_data=[]
            parkedcar_fence = []
            parked_car_line_fences =[]

            dynamic_car = None

            # Parked car(s) (X(m), Y(m), Z(m), Yaw(deg), RADX(m), RADY(m), RADZ(m))
            
            for agent in world.get_actors().filter("vehicle.*"):
                if agent.attributes.get('role_name') == 'hero':
                    continue
                if check_actor_static(get_actor_velocity(agent)):
                    parked_car_info = []
                    parked_car_fence_info=[]
                    parked_car_info.append(agent.get_location().x)
                    parked_car_info.append(agent.get_location().y)
                    parked_car_info.append(agent.get_location().z)
                    parked_car_info.append(agent.get_transform().rotation.yaw)
                    parked_car_fence_info.append(agent.get_location().x)
                    parked_car_fence_info.append(agent.get_location().y)
                    parked_car_fence_info.append(agent.get_location().z)
                    parked_car_fence_info.append(agent.get_transform().rotation.yaw)
                    parked_car_info.append(agent.bounding_box.extent.x)
                    parked_car_info.append(agent.bounding_box.extent.y)
                    parked_car_info.append(agent.bounding_box.extent.z)
                    parkedcar_data.append(parked_car_info)
                    parkedcar_fence.append(parked_car_fence_info)


            # obtain parked car(s) box points for LP ---- Required for nudge cases
            for i in range(len(parkedcar_data)):
                x = parkedcar_data[i][0]
                y = parkedcar_data[i][1]
                z = parkedcar_data[i][2]
                yaw = parkedcar_data[i][3]
                xrad = parkedcar_data[i][4]
                yrad = parkedcar_data[i][5]
                zrad = parkedcar_data[i][6]
                cpos = np.array([
                                [-xrad, -xrad, -xrad, 0,    xrad, xrad, xrad,  0    ],
                                [-yrad, 0,     yrad,  yrad, yrad, 0,    -yrad, -yrad]])
                rotyaw = np.array([
                                  [np.cos(yaw), np.sin(yaw)],
                                  [-np.sin(yaw), np.cos(yaw)]])
                cpos_shift = np.array([
                                      [x, x, x, x, x, x, x, x],
                                      [y, y, y, y, y, y, y, y]])
                cpos = np.add(np.matmul(rotyaw, cpos), cpos_shift)
                for j in range(cpos.shape[1]):
                    parkedcar_box_pts.append([cpos[0,j], cpos[1,j]])
            trajectory_fig.update("parkedcar_pts",
                                         parkedcar_data[0][0], parkedcar_data[0][1],
                                         'r')

            # obtain parked car fence points for LP ---- Required for STOP cases
            for i in range(len(parkedcar_fence)):
                parked_car_wp = map.get_waypoint(carla.Location(parkedcar_fence[i][0],parkedcar_fence[i][1],parkedcar_fence[i][2]))
                #parked_car_wp = parked_car_wp.previous(1.0)[0]
                x = parked_car_wp.transform.location.x + ((parked_car_wp.lane_width/2)* (math.cos(math.radians(iConvertTo360(parked_car_wp.transform.rotation.yaw)+ 90 ))))
                y = parked_car_wp.transform.location.y + ((parked_car_wp.lane_width/2)* (math.sin(math.radians(iConvertTo360(parked_car_wp.transform.rotation.yaw)+ 90 ))))
                z = parked_car_wp.transform.location.z
                yaw = (parked_car_wp.transform.rotation.yaw* np.pi/180) #parkedcar_fence[i][3] + np.pi #/ 2.0  # add 90 degrees for fence
                spos = np.array([
                                [0, 0                       ],
                                [0, STOP_SIGN_FENCELENGTH]])
                rotyaw = np.array([
                                  [np.cos(yaw), np.sin(yaw)],
                                  [-np.sin(yaw), np.cos(yaw)]])
                spos_shift = np.array([
                                      [x, x],
                                      [y, y]])
                spos = np.add(np.matmul(rotyaw, spos), spos_shift)
                parked_car_line_fences.append([spos[0,0], spos[1,0], spos[0,1], spos[1,1]])



            for agent in world.get_actors().filter("vehicle.*"):
                if agent.attributes.get('role_name') == 'hero':
                    continue
                if not (check_actor_static(get_actor_velocity(agent))):
                    dynamic_car = agent
                    dynamic_obstacle_data = [[agent.get_location().x,agent.get_location().y]]


            dynamic_car_path = getActorTrajectories(dynamic_car,map,world)
            expanded_car_path = []
            if dynamic_car_path is not None:
                expanded_car_path = expandActorTrajectories(dynamic_car_path,map)
                expanded_car_path_np = np.array(expanded_car_path)
            dynamic_car_path_np = np.array(dynamic_car_path)
            reached_the_end = False

            current_timestamp = start_timestamp

            prev_timestamp = current_timestamp
            current_x, current_y, current_yaw = get_current_pose(ego)
            current_speed = get_actor_velocity(ego)
            current_timestamp = world.get_snapshot().timestamp.elapsed_seconds


            if frame % LP_FREQUENCY_DIVISOR == 0:
                stop_source_found = False
                # Compute open loop speed estimate.
                open_loop_speed = vp.get_open_loop_speed(current_timestamp - prev_timestamp)

                # Calculate the goal state set in the local frame for the local planner.
                # Current speed should be open loop for the velocity profile generation.
                ego_state = [current_x, current_y, current_yaw, open_loop_speed]

                bp_components = [ego_state,waypoints,parked_car_line_fences,parkedcar_data]
                
                decision ,goal_index = bp.decision_based_on_rule(bp_components)
#                if decision == 'STOP':
#                    stop_source_found = True
#                    goal_index, stop_source_found,intersect_point = stop_planner.check_for_stop_source(waypoints, closest_index, goal_index,parked_car_line_fences)
                goal_state = waypoints[goal_index]

                #print(decision)

                goal_state_set = []
                # Compute the goal state set from the behavioural planner's computed goal state.
                goal_state_set = lp.get_goal_state_set(goal_index, goal_state, waypoints, ego_state)


                # Calculate planned paths in the local frame.
                paths, path_validity = lp.plan_paths(goal_state_set)

                # Transform those paths back to the global frame.
                paths = lp.transform_paths(paths, ego_state)

                # Perform collision checking.
                if decision == 'NUDGE':
                    collision_check_array = cc.collision_check(paths, [parkedcar_box_pts])
                    stop_source_found = check_for_stop(collision_check_array,paths,expanded_car_path)
#                    print(stop_source_found)
                else:
                    collision_check_array = np.ones(len(paths),dtype=bool)


                

                # Compute the best local path.
                best_index = cc.select_best_path_index(paths, collision_check_array, goal_state)

                # If no path was feasible, continue to follow the previous best path.
                if best_index == None:
                    best_path = lp._prev_best_path
                else:
                    best_path = paths[best_index]
                    lp._prev_best_path = best_path

                desired_speed = goal_state[2]
                         
                decelerate_to_stop = stop_source_found
                local_waypoints = vp.compute_velocity_profile(best_path, desired_speed, ego_state, current_speed, decelerate_to_stop)
                
                if local_waypoints != None:
                    wp_distance = []   # distance array
                    local_waypoints_np = np.array(local_waypoints)
                    for i in range(1, local_waypoints_np.shape[0]):
                        wp_distance.append(
                                np.sqrt((local_waypoints_np[i, 0] - local_waypoints_np[i-1, 0])**2 +
                                        (local_waypoints_np[i, 1] - local_waypoints_np[i-1, 1])**2))
                    wp_distance.append(0)  # last distance is 0 because it is the distance
                                           # from the last waypoint to the last waypoint

                    # Linearly interpolate between waypoints and store in a list
                    wp_interp      = []    # interpolated values 
                                           # (rows = waypoints, columns = [x, y, v])
                    for i in range(local_waypoints_np.shape[0] - 1):
                        # Add original waypoint to interpolated waypoints list (and append
                        # it to the hash table)
                        wp_interp.append(list(local_waypoints_np[i]))
                
                        num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                                     float(INTERP_DISTANCE_RES)) - 1)
                        wp_vector = local_waypoints_np[i+1] - local_waypoints_np[i]
                        wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                        for j in range(num_pts_to_interp):
                            next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                            wp_interp.append(list(local_waypoints_np[i] + next_wp_vector))
                    # add last waypoint at the end
                    wp_interp.append(list(local_waypoints_np[-1]))
                    
                    # Update the other controller values and controls
                    controller.update_waypoints(wp_interp)
                    pass

            ###
            # Controller Update
            ###
            if local_waypoints != None and local_waypoints != [] and not stop_source_found:
                controller.update_values(current_x, current_y, current_yaw, 
                                         current_speed,
                                         current_timestamp, frame)
                controller.update_controls()
                cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
            else:
                cmd_throttle = 0.0
                cmd_steer = 0.0
                cmd_brake = 0.0
            #print(cmd_throttle, cmd_steer, cmd_brake)
            # Output controller command to CARLA server


#            trajectory_fig.roll("trajectory", current_x, current_y)
#            trajectory_fig.roll("car", current_x, current_y)
            if ENABLE_PLOT:
                if frame % LP_FREQUENCY_DIVISOR == 0:
                    path_counter = 0
                    for i in range(NUM_PATHS):
                        
                        # If a path was invalid in the set, there is no path to plot.
                        if path_validity[i]:
                            # Colour paths according to collision checking.
                            if not collision_check_array[path_counter]:
                                colour = 'r'
                            elif i == best_index:
                                colour = 'k'
                            else:
                                colour = 'b'
                            trajectory_fig.update("local_path " + str(i), paths[path_counter][0], paths[path_counter][1], colour)
                            path_counter += 1
                        else:
                            trajectory_fig.update("local_path " + str(i), [ego_state[0]], [ego_state[1]], 'r')


                    wp_interp_np = np.array(wp_interp)
                    path_indices = np.floor(np.linspace(0, 
                                                        wp_interp_np.shape[0]-1,
                                                        INTERP_MAX_POINTS_PLOT))


            if ENABLE_PLOT:
                trajectory_fig.update("selected_path", 
                    wp_interp_np[path_indices.astype(int), 0],
                    wp_interp_np[path_indices.astype(int), 1],
                    new_colour=[1, 0.5, 0.0])
                if dynamic_car_path is not None:
                    trajectory_fig.update("dynamiccar_path", 
                        dynamic_car_path_np[0:5,0],
                        dynamic_car_path_np[0:5,1],
                        new_colour=[0, 0, 0.0])

                lp_traj.refresh()

            send_control_command(ego,
                                 throttle=cmd_throttle,
                                 steer=cmd_steer,
                                 brake=cmd_brake)




            # Find if reached the end of waypoint. If the car is within
            # DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint,
            # the simulation will end.
            dist_to_last_waypoint = np.linalg.norm(np.array([
                waypoints[-1][0] - current_x,
                waypoints[-1][1] - current_y]))
            if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                reached_the_end = True
            if reached_the_end:
                break

            landmarks = []
            stopsign_data = []


            parkedcar_data=[]
            parkedcar_fence = []
            parkedcar_box_pts = []      # [x,y]
            parked_car_line_fences =[]

         
        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)
        
        finally:
            pass
            #print("Frame executed")

    print("Reached the end of path.Ending the Scenario...Bye!!")
    send_control_command(ego, throttle=0.0, steer=0.0, brake=1.0)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

