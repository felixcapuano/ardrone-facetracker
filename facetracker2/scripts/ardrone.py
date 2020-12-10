#!/usr/bin/env python

import rospy
import cv2
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ardrone_autonomy.msg import Navdata


class ARDrone:
    empty_msg = Empty()

    navdata = {}

    def __init__(self):

        # rostopic list => publisher
        self.takeoff_pub = rospy.Publisher( "/ardrone/takeoff", Empty, queue_size=10)
        self.landing_pub = rospy.Publisher( "/ardrone/land", Empty, queue_size=10)
        self.move_pub = rospy.Publisher( "/cmd_vel", Twist, queue_size=10)  # Publish commands to drone
        self.reset_pub = rospy.Publisher( "/ardrone/reset", Empty, queue_size=10)

        # image
        self.images_callback = None

    def move(self, speed=[0.0, 0.0, 0.0], orient=[0.0, 0.0, 0.0], period=2):
        print("move for {} seconds [direction={} , orientation={}]".format(
            period, speed, orient))

        vel_msg = Twist()

        vel_msg.linear.x = speed[0]
        vel_msg.linear.y = speed[1]
        vel_msg.linear.z = speed[2]

        vel_msg.angular.x = orient[0]
        vel_msg.angular.y = orient[1]
        vel_msg.angular.z = orient[2]

        self._freeze(period, self.move_pub, vel_msg)

    def onNewImages(self, ic):
        self.images_callback = ic
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( "/ardrone/front/image_raw", Image, self._img_callback)

    def _img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
                print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)
        
        if self.images_callback is not None:
            self.images_callback(cv_image)

    def reset(self):

        self.reset_pub.publish(self.empty_msg)

    def listen_navdata(self):
        self.navdata_sub = rospy.Subscriber("/ardrone/navdata", Navdata, self._navdata_callback)

    def _navdata_callback(self, data):
        print(data.magX)

    def stop(self, period=3):
        self.move(period=period)

    def takeoff(self, period=10):
        print("take off for {} seconds".format(period))

        self._freeze(period, self.takeoff_pub, self.empty_msg)

    def land(self):
        print("landing drone")
        self.landing_pub.publish(self.empty_msg)

    def _freeze(self, period, pub=None, msg=None):
        t = time.time()

        print('freeze')
        while(time.time()-t < period):
            if pub is not None and msg is not None:
                pub.publish(msg)


def callback(cv_image):
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':
    
    rospy.init_node('basic_controller', anonymous=True)
    drone = ARDrone()

    try:

        drone.onNewImages(callback)

        drone.takeoff(period=10)

        speed = [0.1, 0, 0]
        orient = [0, 0, 0]
        drone.move(speed, orient, period=3)

        drone.stop(period=3)

        speed = [-0.1, 0, 0]
        drone.move(speed, orient, period=5)

        drone.stop(period=3)

        drone.land()
    except KeyboardInterrupt:
        drone.land()
    finally:
        drone.land()
