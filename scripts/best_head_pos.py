import cv2
import os
import dlib

dir_path = os.path.dirname(os.path.abspath(__file__))
#path = dir_path + "/../resources/cv2/shape_predictor_68_face_landmarks.dat"
#predictor = dlib.shape_predictor(path) 

detector = dlib.get_frontal_face_detector() 

def get_heads_pos(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    faces = detector(gray) 

    centers = []
    for face in faces:
        x1 = int(face.left())
        y1 = int(face.top())
        x2 = int(face.right())
        y2 = int(face.bottom())

        x = int((x1 + x2)/2)
        y = int((y1 + y2)/2)

        centers.append((x, y))
            
    return centers
