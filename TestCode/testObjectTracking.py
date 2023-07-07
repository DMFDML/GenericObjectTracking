import cv2

def draw_bounding_boxs(img, success, boxes):
    if (success):
        x,y,w,h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x,y), ((x+w), (y+h)), (0, 255, 0), 3, 1)
        cv2.putText(img, "Got", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(img, "Lost", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    

cap = cv2.VideoCapture(0)
# tracker1 = cv2.legacy.TrackerMOSSE_create()
# tracker2 = cv2.TrackerCSRT.create()
success, img = cap.read()

bbox = cv2.selectROI("tracking", img, False)
# tracker1.init(img, bbox)

# bbox = cv2.selectROI("tracking", img, False)
# tracker2.init(img, bbox)


while True:
    timer = cv2.getTickCount()
    success, img = cap.read()

    if (not success):
        break

    # success, bbox = tracker1.update(img)
    print(img.shape, " ", bbox)
    draw_bounding_boxs(img, success, bbox)
    
    # success, bbox = tracker2.update(img)
    # draw_bounding_boxs(img, success, bbox)

    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("tracking", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()