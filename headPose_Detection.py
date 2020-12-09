import cv2


def headpose_detect_func(frame):
    
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
  
    faces = face_cascade.detectMultiScale(frame, 1.3, 5)
        
    for (x,y,w,h) in faces:
        gray = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        gray = cv2.circle(frame, (int(x+w/2),int(y+h/2)), 5,(0, 0, 255), 2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = gray[y:y+h, x:x+w]
     
    # Display the resulting frame
    cv2.imshow('frame',frame)
    