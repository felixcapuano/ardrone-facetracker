#!/usr/bin/env python

import rospy
rospy.init_node('controller', anonymous=True)
from geometry_msgs.msg import Point
from ardrone import ARDrone
import threading
from cv2 import waitKey



SPEED = 0.05
linear = [0, 0, 0]
angular = [0, 0, 0]

lock = threading.Lock()

def ft_callback(v):
    global linear
    global lock
    print(linear)
    with lock:
        linear = [v.x*SPEED, v.y*SPEED, v.z*SPEED]


ft_sub = rospy.Subscriber('facetracker', Point, ft_callback)

drone = ARDrone(verbose=True)
drone.listen_navdata()
drone.takeoff()



def worker_controller(lock):   
    while True:
        global linear
        vertical_linear = [0, 0, 0]
        horizontal_linear = [0, 0, 0]
        with lock:
            
            horizontal_linear = [0, linear[1], 0]
            drone.move(linear, horizontal_linear, period=1)

            
            vertical_linear = [0, 0, linear[2]]
            drone.move(linear, vertical_linear, period=1)

            linear = [0, 0, 0]

            if waitKey(1) and 0xFF == ord('q'):
                drone.land()
                break


controller_thread = threading.Thread(target=worker_controller, args=(lock,))
controller_thread.start()
#raw_input("type enter to land")
#drone.land()


