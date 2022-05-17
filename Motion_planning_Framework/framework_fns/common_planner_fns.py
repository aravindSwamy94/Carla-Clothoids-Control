import numpy as np


def get_closest_index(waypoints, ego_state):
    closest_len = float('Inf')
    closest_index = 0
    for i in range(len(waypoints)):
        temp = (waypoints[i][0] - ego_state[0])**2 + (waypoints[i][1] - ego_state[1])**2
        if temp < closest_len:
            closest_len = temp
            closest_index = i
    closest_len = np.sqrt(closest_len)

    return closest_len, closest_index



def get_goal_index(waypoints, ego_state, closest_len, closest_index,lookahead_dist):

    arc_length = closest_len
    wp_index = closest_index
       
    if arc_length > lookahead_dist:
        return wp_index

    if wp_index == len(waypoints) - 1:
        return wp_index
    while wp_index < len(waypoints) - 1:
        arc_length += np.sqrt((waypoints[wp_index][0] - waypoints[wp_index+1][0])**2 + (waypoints[wp_index][1] - waypoints[wp_index+1][1])**2)
        if arc_length > lookahead_dist: break
        wp_index += 1

    return wp_index

