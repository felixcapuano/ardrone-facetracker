#!/usr/bin/env python

import cv2
import numpy as np
from simple_head_pos import get_heads_pos

def get_correction_dir(cv_image):
    heads_pos = get_heads_pos(cv_image)
    # peut etre faire la moyenne si deux tete sont detecté
    if len(heads_pos) > 0:
        w_head_pos, h_head_pos = heads_pos[0]

        height, width, channels = cv_image.shape
        w_img_center, h_img_center = width//2, height//2

        # vecteur direction du centre par rapport a la position de la tête
        vect = np.array([w_img_center-w_head_pos, h_head_pos-h_img_center])

        # normalisation du vecteur
        vect_norm = vect / np.sqrt(np.sum(vect**2))

        print(vect_norm)
        return vect_norm

if __name__ == '__main__':
    import rospy
    rospy.init_node('basic_controller', anonymous=True)
    from ardrone import ARDrone
    drone = ARDrone(verbose=True)

    drone.listen_image(get_correction_dir)

    # temp
    drone.listen_navdata()
    drone.takeoff()
    drone.stop()
    
    raw_input("type enter to land")
    drone.land()
