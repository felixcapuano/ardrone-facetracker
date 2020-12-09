import cv2
from headPose_Detection import headpose_detect_func

cap = cv2.VideoCapture(0)
    
while(True):
    ret, frame = cap.read()
    headpose_detect_func(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()