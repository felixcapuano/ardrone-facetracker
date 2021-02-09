#!/usr/bin/env python

import rospy
from ardrone import ARDrone
from readchar import readkey, key

rospy.init_node('controller_node', anonymous=True)
drone = ARDrone(verbose=True)
drone.listen_navdata()

speed = 0.2
linear = [0, 0, 0]
angular = [0, 0, 0]
while True:
    k = readkey()
    if k == 'z':
        linear = [speed, 0, 0]
        angular = [0, 0, 0]
    elif k == 's':
        linear = [-speed, 0, 0]
        angular = [0, 0, 0]
    elif k == 'd':
        linear = [0, -speed, 0]
        angular = [0, 0, 0]
    elif k == 'q':
        linear = [0, speed, 0]
        angular = [0, 0, 0]
    elif k == key.LEFT:
        linear = [0, 0, 0]
        angular = [0, 0, speed]
    elif k == key.RIGHT:
        linear = [0, 0, 0]
        angular = [0, 0, -speed]
    elif k == key.UP:
        linear = [0, 0, speed]
        angular = [0, 0, 0]
    elif k == key.DOWN:
        linear = [0, 0, -speed]
        angular = [0, 0, 0]
    elif k == key.SPACE:
        linear = [0, 0, 0]
        angular = [0, 0, 0]
    elif k == 't':
        drone.takeoff()
    elif k == 'l':
        drone.land()
    elif k == key.SUPR:
        drone.land()
        print('QUIT')
        break
    elif k == 'h':
        print('''
help        h
quit        suppr

-State:
takeoff     t
land        l
hover/stop  space

-Translation:
up          up_arrow
down        down_arrow
forward     z
backward    s
left        q
right       d

-Rotation:
left        left_arrow
right       right_arrow
        ''')

    drone.move(linear, angular, period=0)

