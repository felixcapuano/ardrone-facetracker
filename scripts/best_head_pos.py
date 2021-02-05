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

    center = None
    max_face_size = 0
    face_target = None
    for face in faces:
        face_left = int(face.left())
        face_right = int(face.right())
        current_face_size = abs(face_right-face_left)

        if current_face_size > max_face_size:
            max_face_size = current_face_size
            face_target = face

    x, y = 0, 0
    if face_target != None:
        x1 = int(face_target.left())
        y1 = int(face_target.top())
        x2 = int(face_target.right())
        y2 = int(face_target.bottom())

        x = int((x1 + x2)/2)
        y = int((y1 + y2)/2)
            
    return (x, y, max_face_size)
