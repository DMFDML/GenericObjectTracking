import cv2
import numpy as np

# Outputs the point cloud as a json file
def outputPCImage(capture, no):
    cv2.imwrite(f"images/calibration/calibrate{no}.png", capture)
    print(f"writing image images/calibration/calibrate{no}.png")

cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# tracker = Sort_OH(max_age=30)


save = 0

while cap.isOpened():
    ret, img = cap.read()
    
    cv2.imshow('Image', img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('s'):
        
        outputPCImage(img, save)
        save+=1

cap.release()
cv2.destroyAllWindows()