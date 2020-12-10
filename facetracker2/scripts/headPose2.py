# We Import the necessary packages needed 
import cv2 
from face_detection_func import face_detection


cap = cv2.VideoCapture(0) 
 
# Start the main program 
while True: 
    try:
        _, frame = cap.read()  
        face_detection(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break # press esc the frame is destroyed
    except:
        pass

cap.release()
cv2.destroyAllWindows()