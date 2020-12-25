import numpy as np
import cv2
import os

dir_path = os.path.dirname(os.path.abspath(__file__))

xml_data = dir_path + "/../resources/cv2/haarcascade_frontalface_default.xml"
print(xml_data)
face_cascade = cv2.CascadeClassifier(xml_data)

def face_detection(frame):

    faces = face_cascade.detectMultiScale(frame, 1.3, 5)

    for (x,y,w,h) in faces:
        gray = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = gray[y:y+h, x:x+w]
            
    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.waitKey(1)
