import cv2

def headpose_detect_func(frame):
    
    diff_x = None
    diff_y = None
    
    (height, weight) = frame.shape[:2]
    center_image_x = weight/2
    center_image_y = height/2
    
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(frame, 1.3, 5)
       
    for (x,y,w,h) in faces:
        center_x = int(x+w/2)
        center_y = int(y+h/2)
        
        gray = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        
        gray = cv2.circle(frame, (center_x,center_y), 5,(0, 0, 255), 2)
        gray = cv2.circle(frame,(int(center_image_x),int(center_image_y)), 20,(0, 0, 255), 2)
          
        diff_x = center_x - center_image_x
        diff_y = center_y - center_image_y
              
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = gray[y:y+h, x:x+w]
    
    # Display the resulting frame
    cv2.imshow('frame',frame)
    
    return diff_x, diff_y
    
def detect_position(diff_x, diff_y):
    
    if (diff_x < -20):
            print('Fonction : Bouger Drone vers la droite')
            # move_drone_X(vitesse = -k)
           
    elif (diff_x > 20):
        print('Fonction : Bouger Drone vers la gauche')
        # move_drone_X(vitesse = +k)
              
    elif (-20 < diff_x < 20):
        print('Drone centré sur X')

        if (diff_y < -20):
            print('Fonction : Bouger Drone vers le bas')
            # move_drone_Y(vitesse = -k)
            
        elif (diff_y > 20):
            print('Fonction : Bouger Drone vers le haut')
            # move_drone_X(vitesse = +k)       
                           
        elif (-20 < diff_y < 20):
            print('Drone centré sur X et Y')
            # drone_stop_moving(vitesse = 0)
    
    