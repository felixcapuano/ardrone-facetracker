import cv2
import sys
from facetracker_node import process_img

cap = cv2.VideoCapture(0) 
 
# Start the main program 
while True: 
    try:
        _, frame = cap.read()

        process_img(frame)
    except KeyboardInterrupt:
        print("interupted")
        break
    except Exception as erreur:
        print(erreur)

cap.release()
cv2.destroyAllWindows()
