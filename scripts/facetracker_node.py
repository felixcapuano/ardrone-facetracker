#!/usr/bin/env python

import rospy
rospy.init_node('basic_controller', anonymous=True)
import cv2
from ardrone import ARDrone
drone = ARDrone(verbose=True)

def detect_face(cv_image):
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)

drone.listen_image(detect_face)

if __name__ == '__main__':
    # temp
    drone.listen_navdata()
    drone.takeoff()
    drone.stop()
    
    raw_input("type key to land")
    drone.land()
