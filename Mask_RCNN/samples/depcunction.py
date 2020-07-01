import os
import numpy as np
import cv2
import maskrcnn


if __name__=='__main__':
    capture=cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    while(1):
        ret,img=capture.read()
        maskrcnn.model_detection(img)
        cv2.imshow('frame',img)
        if cv2.waitKey(1)&0xFF==ord('q'):
            break

    cv2.destroyAllWindows()
