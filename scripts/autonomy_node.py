#!/usr/bin/env python

import rospy
rospy.init_node('controller', anonymous=True)
from geometry_msgs.msg import Point
from ardrone import ARDrone
import time


time_speed = 1
move_speed = 0.1
p = 1

def controller(vector):   

    linear = [ 0, 0, -move_speed*vector.z]
    angular = [0, 0, move_speed*vector.y]
    drone.move(linear, angular, period=p)

    drone.stop(period=0)

def ft_callback(v):
    if not (v.x==0 and v.y==0 and v.z==0):
        controller(v)

drone = ARDrone(verbose=True)
drone.listen_navdata()

ft_sub = rospy.Subscriber('facetracker',
                            Point,
                            ft_callback,
                            queue_size=1)

rospy.spin()