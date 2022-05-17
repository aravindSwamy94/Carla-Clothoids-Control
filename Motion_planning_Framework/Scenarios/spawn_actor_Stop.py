import glob
import os
import sys

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
import random


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    map = world.get_map()
    vehicle_bp = random.choice(blueprints.filter('vehicle.audi.*'))
#Transform(Location(x=396.073395, y=226.690994, z=-0.005038), Rotation(pitch=0.100773, yaw=-90.007576, roll=0.000081))

    
    spawn_transform= carla.Transform(carla.Location(x=396.473395, y=226.690994, z=2), carla.Rotation(pitch=0.100773, yaw=-90.007576, roll=0.000081))
#    actor = world.try_spawn_actor(vehicle_bp,map.get_waypoint(spawn_transform.location).transform)
    actor = world.try_spawn_actor(vehicle_bp,spawn_transform)



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
