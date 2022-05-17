from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import csv
import time
import weakref
import controller2d
import configparser
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
#    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
#    sys.path.append(os.path.abspath(sys.path[0] + '../Course4FinalProject/'))
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
from agents.navigation.roaming_agent import *
from agents.navigation.basic_agent import *

def get_actor_velocity(Actor):
	velX = Actor.get_velocity().x * Actor.get_velocity().x
	velY = Actor.get_velocity().y * Actor.get_velocity().y
	velZ = Actor.get_velocity().z * Actor.get_velocity().z
	return math.sqrt(velX+velY+velZ)		

def main():
	try:
		client = carla.Client('localhost', 2000)
		client.set_timeout(10.0)
		world = client.get_world()
		map = world.get_map()
		blueprint_library = world.get_blueprint_library()
		for actor in world.get_actors().filter("vehicle.*"):
			if actor.attributes.get('role_name') == 'hero':
				ego = actor
				break
#		print(world.get_weather())
#		while(1):#
#			print(map.get_waypoint(ego.get_transform().location).transform.rotation.yaw)
		print(ego.get_transform())
		
	finally:
		print("Bye..!!!")
		
if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
