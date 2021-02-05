#!/usr/bin/env python

import rospy
rospy.init_node('controller', anonymous=True)
from geometry_msgs.msg import Quaternion
from ardrone import ARDrone
import time


move_period = 1

move_speed = 0.1
rot_speed = 0.1

x_move_speed = 0.1

def controller(vector):   

    x_move = 0
    if vector.w < 80:
        x_move = x_move_speed
    elif vector.w > 90:
        x_move = -x_move_speed
        

    linear = [ x_move, 0, -move_speed*vector.z]
    angular = [ x_move, 0, rot_speed*vector.y]
    drone.move(linear, angular, period=move_period)

    drone.stop(period=0)

def ft_callback(v):
    if not (v.x==0 and v.y==0 and v.z==0):
        controller(v)

drone = ARDrone(verbose=True)
drone.listen_navdata()

ft_sub = rospy.Subscriber('facetracker',
                            Quaternion,
                            ft_callback,
                            queue_size=1)

rospy.spin()