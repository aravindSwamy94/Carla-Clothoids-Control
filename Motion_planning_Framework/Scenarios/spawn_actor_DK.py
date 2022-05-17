import glob
import os
import sys
import math
try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import numpy as np
import random
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

def get2Ddistance(ego_car_location,ego_spawn_trigger):
    diffX = (ego_spawn_trigger.x - ego_car_location.x) * (ego_spawn_trigger.x - ego_car_location.x)
    diffY = (ego_spawn_trigger.y - ego_car_location.y) * (ego_spawn_trigger.y - ego_car_location.y)
    absoluteDistance = math.sqrt(diffX + diffY)
    return absoluteDistance


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    map = world.get_map()
    vehicle_bp = random.choice(blueprints.filter('vehicle.audi.etron'))

    vehicle_bp_stop = random.choice(blueprints.filter('vehicle.audi.*'))
#Transform(Location(x=396.073395, y=226.690994, z=-0.005038), Rotation(pitch=0.100773, yaw=-90.007576, roll=0.000081))

    
    spawn_transform_stop= carla.Transform(carla.Location(x=396.473395, y=226.690994, z=2), carla.Rotation(pitch=0.100773, yaw=-90.007576, roll=0.000081))
#    actor = world.try_spawn_actor(vehicle_bp,map.get_waypoint(spawn_transform.location).transform)
    actor_stop = world.try_spawn_actor(vehicle_bp_stop,spawn_transform_stop)

    ego_trigger = carla.Transform(carla.Location(x=396.204010, y=265.395889, z=-0.002537), carla.Rotation(pitch=0.098806, yaw=-91.021294, roll=-0.000122))

    spawn_transform= carla.Transform(carla.Location(x=392.141022, y=217.984726, z=0.5), carla.Rotation(pitch=0.126666, yaw=92.436951, roll=0.000076))    

# lead vehicle stop location Transform(Location(x=396.069183, y=147.905167, z=-0.005042), Rotation(pitch=0.110588, yaw=-90.001984, roll=-0.000031))
    lead_stop=carla.Transform(carla.Location(x=396.069183, y=147.905167, z=-0.005042), carla.Rotation(pitch=0.110588, yaw=-90.001984, roll=-0.000031))
    
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'hero':
            ego = actor
            break
    while(1):
        if get2Ddistance(ego.get_location(),ego_trigger.location) < 10:
            break
    lead_actor = world.spawn_actor(vehicle_bp,spawn_transform)
    lead_actor.set_autopilot(True)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
