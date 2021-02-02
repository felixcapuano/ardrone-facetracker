#!/usr/bin/env python

import cv2
import numpy as np
from simple_head_pos import get_heads_pos

TARGET_SIZE = 20

def in_target(img_centre, pt):
    if img_centre[0]+TARGET_SIZE > pt[0] and \
        img_centre[0]-TARGET_SIZE < pt[0] and \
        img_centre[1]+TARGET_SIZE > pt[1] and \
        img_centre[1]-TARGET_SIZE < pt[1]:
        return True
    else:
        return False

def process_img(cv_image):
    vect_norm = np.zeros(3)

    # on cherche la position des tetes sur l'image
    heads_pos = get_heads_pos(cv_image)

    # on calcule le centre de l'image
    height, width, _ = cv_image.shape
    img_center = (width//2, height//2)

    # affichage de la cible
    cv2.rectangle(cv_image,
                (img_center[0]-TARGET_SIZE, img_center[1]-TARGET_SIZE),
                (img_center[0]+TARGET_SIZE, img_center[1]+TARGET_SIZE),
                (0, 0, 255), 1)

    # si aucune tete n'est detectee
    if len(heads_pos) > 0:
        # je prend seulement la premier tete detecter ici
        # peut etre faire la moyenne si deux tete sont detecte
        head_pos = heads_pos[0]

        # si la tete est suffisament centree
        if not in_target(img_center, head_pos):
            # vecteur direction du centre par rapport a la position de la tete
            vect = np.array([0, img_center[0]-head_pos[0], head_pos[1]-img_center[1]])
            # normalisation du vecteur
            vect_norm = vect / np.sqrt(np.sum(vect**2))

        # affichage du vecteur
        cv2.arrowedLine(cv_image,
                        (img_center[0], img_center[1]),
                        (head_pos[0], head_pos[1]),
                        (255, 0, 0), 2)

    # affiche l'image
    cv2.imshow('camera', cv_image)
    cv2.waitKey(1)
    
    return vect_norm

if __name__ == '__main__':
    import rospy
    rospy.init_node('face_tracker', anonymous=True)
    from geometry_msgs.msg import Point
    from ardrone import ARDrone


    vect_pub = rospy.Publisher('facetracker', Point, queue_size=10)
    def image_callback(img):
        vect = process_img(img)
        print("publish :", vect) 

        # publish HERE
        vect_pub.publish(*vect)

    drone = ARDrone(verbose=True)
    drone.listen_image(image_callback)

    rospy.spin()
