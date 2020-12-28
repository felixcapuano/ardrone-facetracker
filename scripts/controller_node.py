#!/usr/bin/env python

import rospy
rospy.init_node('controller', anonymous=True)
from geometry_msgs.msg import Point
from ardrone import ARDrone

SPEED = 0.3

drone = ARDrone(verbose=True)
drone.listen_navdata()
drone.takeoff()

def ft_callback(v):
    linear = [v.x*SPEED, v.y*SPEED, v.z*SPEED]
    angular = [0, 0, 0] 
    print(linear)
    
    drone.move(linear, angular, period=0)

ft_sub = rospy.Subscriber('facetracker', Point, ft_callback)

raw_input("type enter to land")
drone.land()


