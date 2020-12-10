#!/usr/bin/env python

import cv2
import rospy
from ardrone import ARDrone

from face_detection_func import face_detection
#from face_detection_funcadv import face_detection


if __name__ == '__main__':

    def callback(cv_image):
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    
    rospy.init_node('basic_controller', anonymous=True)
    drone = ARDrone()

    try:
        drone.listen_navdata()
        drone.onNewImages(face_detection)
        rospy.spin()

        #drone.takeoff(period=10)
        #speed = [0, 0, 0.05]
        #orient = [0, 0, 0.05]
        #drone.move()

        #speed = [0.1, 0, 0]
        #orient = [0, 0, 0]
        #drone.move(speed, orient, period=2)

        #drone.stop(period=2)

        #speed = [-0.1, 0, 0]
        #orient = [0, 0, 0]
        #drone.move(speed, orient, period=2)
        #drone.stop(period=2)

        #drone.land()

    finally:
        drone.land()
        pass
