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

    verbose = True

    # drone state
    UNKNOWN     = [0 ]
    INITED      = [1 ]
    LANDED      = [2 ]
    FLYING      = [3, 7 ]
    HOVERING    = [4 ]
    TEST        = [5 ]
    TAKINGOFF   = [6 ]
    LANDING     = [8 ]
    LOOPING     = [9 ]

    empty_msg = Empty()

    navdata = None
    navdata_ready = False

    images_callback = None
    navdata_callback = None

    def __init__(self, verbose=True):
        self.verbose = verbose

        # rostopic list => publisher
        self.takeoff_pub = rospy.Publisher( "/ardrone/takeoff", Empty, queue_size=10)
        self.landing_pub = rospy.Publisher( "/ardrone/land", Empty, queue_size=10)
        self.move_pub = rospy.Publisher( "/cmd_vel", Twist, queue_size=10)  # Publish commands to drone
        self.reset_pub = rospy.Publisher( "/ardrone/reset", Empty, queue_size=10)

    def move(self, linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0], period=2):
        if self.navdata.state in self.FLYING:
            self.debug("MOVE: {} seconds [direction={} , orientation={}]".format(
                period, linear, angular))

            vel_msg = Twist()

            vel_msg.linear.x = linear[0]
            vel_msg.linear.y = linear[1]
            vel_msg.linear.z = linear[2]

            vel_msg.angular.x = angular[0]
            vel_msg.angular.y = angular[1]
            vel_msg.angular.z = angular[2]

            
            self.move_pub.publish(vel_msg)

            t = time.time()
            while(time.time()-t < period):
                self.move_pub.publish(vel_msg)
        else:
            self.debug("WARN: drone is not in flying state")

    def listen_image(self, ic):
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

    def listen_navdata(self, nc=None):
        self.navdata_callback = nc

        self.navdata_sub = rospy.Subscriber("/ardrone/navdata", Navdata, self._navdata_callback)

        self.debug("Waiting for navigation data")
        while self.navdata_ready == False:
            pass
        self.debug("Navigation data ready")

    def _navdata_callback(self, data):
        self.navdata = data
        self.navdata_ready = True

        if self.navdata_callback is not None:
            self.navdata_callback(data)

    def stop(self, period=3):
        self.move(period=period)

    def takeoff(self):
        self.debug("TAKE OFF: starting")

        while self.navdata.state not in self.TAKINGOFF:
            self.takeoff_pub.publish(self.empty_msg)

        self.debug("TAKE OFF: in progress")
        while self.navdata.state in self.TAKINGOFF:
            pass
        self.debug("TAKE OFF: end")

    def land(self):
        print("LANDING: ok")
        #while self.navdata.state not in self.LANDING:
        self.landing_pub.publish(self.empty_msg)

    def debug(self, info_str):
        if self.verbose == True:
            print(info_str)


if __name__ == '__main__':

    from readchar import readkey, key

    rospy.init_node('basic_controller', anonymous=True)
    drone = ARDrone(verbose=True)
    drone.listen_navdata()
    drone.takeoff()

    speed = 0.2
    linear = [0, 0, 0]
    angular = [0, 0, 0]
    while True:
        k = readkey()

        if k == key.UP:
            print('forward')
            linear = [speed, 0, 0]
        elif k == key.DOWN:
            print('backward')
            linear = [-speed, 0, 0]
        elif k == key.RIGHT:
            print('right')
            linear = [0, -speed, 0]
        elif k == key.LEFT:
            print('left')
            linear = [0, speed, 0]
        elif k == key.LEFT:
            print('left')
            linear = [0, speed, 0]
        elif k == key.LEFT:
            print('left')
            linear = [0, speed, 0]
        elif k == 'u':
            print('up')
            linear = [0, 0, speed]
        elif k == 'd':
            print('down')
            linear = [0, 0, -speed]
        elif k == key.SPACE:
            print('stop')
            linear = [0, 0, 0]
        elif k == 'q':
            print('land')
            drone.land()
            break

        drone.move(linear, angular, period=0)

