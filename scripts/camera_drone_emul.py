import cv2
from face_detection_func import face_detection
from facetracker_node import detect_face

cap = cv2.VideoCapture(0) 
 
# Start the main program 
while True: 
    try:
        _, frame = cap.read()  

        detect_face(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break # press esc the frame is destroyed
    except:
        pass

cap.release()
cv2.destroyAllWindows()