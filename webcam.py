import numpy as np
import cv2 as cv

cap = cv.VideoCapture(0)

cap.set(cv.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,240)

fps = cap.get(cv.CAP_PROP_FPS)
print(fps)

if not cap.isOpened():
    print('Cannot Open Camera')
    exit()


while True:
    ret,frame = cap.read()

    if not ret:
        print("Can't recieve frame (stream end?). Exiting...")
        break

    faceDetection(frame)

    cv.imshow('Logitech Webcam',frame)
    if cv.waitKey(1) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()