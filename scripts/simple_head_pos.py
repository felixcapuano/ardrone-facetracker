import numpy as np
import cv2
import os

dir_path = os.path.dirname(os.path.abspath(__file__))

xml_data = dir_path + "/../resources/cv2/haarcascade_frontalface_default.xml"
face_cascade = cv2.CascadeClassifier(xml_data)

def get_heads_pos(frame):

    faces = face_cascade.detectMultiScale(frame, 1.3, 5)
    centers = []

    for (x,y,w,h) in faces:
        centers.append((x + w//2, y + h//2))
            
    return centers
