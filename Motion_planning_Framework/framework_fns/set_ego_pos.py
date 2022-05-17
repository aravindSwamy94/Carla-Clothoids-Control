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

				

def main():
	try:
		client = carla.Client('localhost', 2000)
		client.set_timeout(10.0)
		world = client.get_world()
		map = world.get_map()
		blueprint_library = world.get_blueprint_library()
		for actor in world.get_actors():
			if actor.attributes.get('role_name') == 'hero':
				ego = actor
				break
		ego.set_transform(carla.Transform(carla.Location(x=396.251892, y=302.294495, z=-0.004815), carla.Rotation(pitch=0.093000, yaw=-89.927544, roll=0.000347)))# Setting the ego vehicle to a set source psition
		ego.set_velocity(carla.Vector3D(0,0,0))
		control = carla.VehicleControl()
		# Clamp all values within their limits
		control.steer = 0
		control.throttle = 0
		control.brake = 0
		control.hand_brake = 0
		control.reverse = 0
		ego.apply_control(control)
		
	finally:
		print("Bye..!!!")
		
if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
