#!/usr/bin/env python

import cv2
import numpy as np
#from simple_head_pos import get_heads_pos
from best_head_pos import get_heads_pos

TARGET_SIZE = 10

def in_target(img_centre, pt):
    if img_centre[0]+TARGET_SIZE > pt[0] and \
        img_centre[0]-TARGET_SIZE < pt[0] and \
        img_centre[1]+TARGET_SIZE > pt[1] and \
        img_centre[1]-TARGET_SIZE < pt[1]:
        return True
    else:
        return False

def process_img(cv_image):
    data = np.zeros(4)

    # on cherche le centre de l'image
    height, width, _ = cv_image.shape
    img_center = (width//2, height//2)

    # affichage de la cible
    cv2.rectangle(cv_image,
                (img_center[0]-TARGET_SIZE, img_center[1]-TARGET_SIZE),
                (img_center[0]+TARGET_SIZE, img_center[1]+TARGET_SIZE),
                (0, 0, 255), 1)

    # on cherche la position des tetes sur l'image
    face_pos_x, face_pos_y, face_size = get_heads_pos(cv_image)

    # si une tete n'est detectee
    if face_size != 0:
        # je prend seulement la premier tete detecter ici
        # peut etre faire la moyenne si deux tete sont detecte

        # si la tete est suffisament centree
        if not in_target(img_center, (face_pos_x, face_pos_y)):
            # vecteur direction du centre par rapport a la position de la tete
            vect = np.array([0, img_center[0]-face_pos_x, face_pos_y-img_center[1], 0])
            # normalisation du vecteur
            data = vect / np.sqrt(np.sum(vect**2))

        # affichage du vecteur
        cv2.arrowedLine(cv_image,
                        (img_center[0], img_center[1]),
                        (face_pos_x, face_pos_y),
                        (255, 0, 0), 2)

    # affiche l'image
    cv2.imshow('camera', cv_image)
    cv2.waitKey(1)
    
    # on ajoute la taille du visage
    data[3] = face_size
    
    return data

if __name__ == '__main__':
    import rospy
    rospy.init_node('face_tracker', anonymous=True)
    from geometry_msgs.msg import Quaternion
    from ardrone import ARDrone

    vect_pub = rospy.Publisher('facetracker', Quaternion, queue_size=10)

    def image_callback(img):
        msg = process_img(img)

        # publish HERE
        if msg[3] != 0:
            print("publish :", msg) 
            vect_pub.publish(*msg)

    drone = ARDrone(verbose=True)
    drone.listen_image(image_callback)

    rospy.spin()
