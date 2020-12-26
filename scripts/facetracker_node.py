#!/usr/bin/env python

import cv2
from face_detection_func import face_detection

def detect_face(cv_image):
    centers = face_detection(cv_image)

if __name__ == '__main__':
    import rospy
    rospy.init_node('basic_controller', anonymous=True)
    from ardrone import ARDrone
    drone = ARDrone(verbose=True)

    drone.listen_image(detect_face)

    # temp
    drone.listen_navdata()
    drone.takeoff()
    drone.stop()
    
    raw_input("type enter to land")
    drone.land()
