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
    vehicle_bp = random.choice(blueprints.filter('vehicle.bmw.isetta'))
#    vehicle_bp = random.choice(blueprints.filter('vehicle.audi.a2'))
#Transform(Location(x=397.474182, y=238.845734, z=-0.005332), Rotation(pitch=0.152177, yaw=-90.907951, roll=-0.000183))

    spawn_transform= carla.Transform(carla.Location(x=397.474182, y=238.845734, z=2), carla.Rotation(pitch=0.152177, yaw=-90.907951, roll=-0.000183))
    actor = world.try_spawn_actor(vehicle_bp,spawn_transform)



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
