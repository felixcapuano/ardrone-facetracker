#!/usr/bin/env python

import rospy
rospy.init_node('controller', anonymous=True)
from geometry_msgs.msg import Point
from ardrone import ARDrone
import time


time_speed = 0.5
move_speed = 0.1

def controller(linear, angular):   

    vertical_linear = [0, 0, 0]
    horizontal_linear = [0, 0, 0]
    
    p_y = abs(linear[1] * time_speed)
    if linear[1] > 0:
        horizontal_linear = [0, move_speed, 0]
    else:
        horizontal_linear = [0, -move_speed, 0]
    drone.move(horizontal_linear, angular, period=p_y)

    p_z = abs(linear[2] * time_speed)
    if linear[2] < 0:
        vertical_linear = [0, 0, move_speed]
    else:
        vertical_linear = [0, 0, -move_speed]
    drone.move(vertical_linear, angular, period=p_z)

    drone.stop(period=0)

def ft_callback(v):
    angular = [0, 0, 0]
    linear = [v.x, v.y, v.z]

    if linear != [0, 0, 0]:
        controller(linear, angular)

drone = ARDrone(verbose=True)
drone.listen_navdata()
drone.takeoff()

ft_sub = rospy.Subscriber('facetracker',
                            Point,
                            ft_callback,
                            queue_size=1)

rospy.spin()

#raw_input("type enter to land")
drone.land()


