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
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get('role_name') == 'hero':
            continue
        lead_car = actor
    lead_car.destroy()




if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
