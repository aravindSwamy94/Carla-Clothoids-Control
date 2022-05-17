import numpy as np
import math


def check_for_lead_vehicle(ego_state, lead_car_position,follow_lead_vehicle_lookahead,follow_lead_vehicle):
    if not follow_lead_vehicle:
        lead_car_delta_vector = [lead_car_position[0] - ego_state[0], 
                                 lead_car_position[1] - ego_state[1]]
        lead_car_distance = np.linalg.norm(lead_car_delta_vector)
        if lead_car_distance > follow_lead_vehicle_lookahead:
            return

        lead_car_delta_vector = np.divide(lead_car_delta_vector, 
                                          lead_car_distance)
        ego_heading_vector = [math.cos(ego_state[2]), 
                              math.sin(ego_state[2])]
        if np.dot(lead_car_delta_vector, 
                  ego_heading_vector) < (1 / math.sqrt(2)):
            return
        follow_lead_vehicle = True

    else:
        lead_car_delta_vector = [lead_car_position[0] - ego_state[0], 
                                 lead_car_position[1] - ego_state[1]]
        lead_car_distance = np.linalg.norm(lead_car_delta_vector)

        if lead_car_distance < follow_lead_vehicle_lookahead + 15:
            return
        lead_car_delta_vector = np.divide(lead_car_delta_vector, lead_car_distance)
        ego_heading_vector = [math.cos(ego_state[2]), math.sin(ego_state[2])]
        if np.dot(lead_car_delta_vector, ego_heading_vector) > (1 / math.sqrt(2)):
            return

        follow_lead_vehicle = False
    return follow_lead_vehicle
