import numpy as np
import math
import follow_planner_fns as follow_planner
import stop_planner_fns as stop_planner
import common_planner_fns as common_planner


NUDGE_CONDITION_THRESHOLD = 0.5
DK_TIME_THRESHOLD = 2
LEAD_VEHICLE_LOOKAHEAD = 20.0 
BP_LOOKAHEAD_BASE      = 16.0            
BP_LOOKAHEAD_TIME      = 1.0                          

def get2dDistance(a,b):
    return math.sqrt((a[0]-b[0])*(a[0]-b[0]) +(a[1]-b[1])*(a[1]-b[1]))

def decision_based_on_rule(bp_components):

    ego_state = bp_components[0]
    waypoints=bp_components[1]
    parked_car_line_fences=bp_components[2]
    parkedcar_data= bp_components[3]

    closest_len, closest_index = common_planner.get_closest_index(waypoints, ego_state)

    goal_index = common_planner.get_goal_index(waypoints, ego_state, closest_len, closest_index,BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * ego_state[3])

    while waypoints[goal_index][2] <= 0.1: goal_index += 1
    
    unchanged_goal_index = goal_index
    
    goal_index, stop_source_found,intersect_point = stop_planner.check_for_stop_source(waypoints, closest_index, goal_index,parked_car_line_fences)

    if stop_source_found:
#        if check_for_stop_or_nudge(parkedcar_data,intersect_point):
#            return "STOP",goal_index
#        else:
        goal_index = unchanged_goal_index
        return "NUDGE",goal_index
    return "LANE_FOLLOW",goal_index


def check_for_stop_or_nudge(parkedcar_data,intersect_point):
    for i in range(len(parkedcar_data)):
        x = parkedcar_data[i][0]
        y = parkedcar_data[i][1]
        z = parkedcar_data[i][2]
        
        intercept_distance = get2dDistance([x,y],intersect_point)
        

        if intercept_distance < NUDGE_CONDITION_THRESHOLD:
            return True
        else:
            return False  

