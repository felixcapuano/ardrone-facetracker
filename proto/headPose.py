import cv2
from headPose_Detection import headpose_detect_func, detect_position

cap = cv2.VideoCapture(0)

ret, frame = cap.read() 

while(True):
    ret, frame = cap.read()
       
    diff_x, diff_y = headpose_detect_func(frame)
    
    if diff_x is not None and diff_y is not None:
        detect_position(diff_x, diff_y)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()