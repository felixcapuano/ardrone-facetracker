#!/usr/bin/env python

import rospy
rospy.init_node('basic_controller', anonymous=True)
from ardrone import ARDrone
drone = ARDrone(verbose=True)

import cv2
from face_detection_func import face_detection


def detect_face(cv_image):
    face_detection(cv_image)

drone.listen_image(detect_face)

if __name__ == '__main__':
    # temp
    drone.listen_navdata()
    drone.takeoff()
    drone.stop()
    
    raw_input("type enter to land")
    drone.land()
