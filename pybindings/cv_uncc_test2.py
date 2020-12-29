#!/usr/bin/env python
import cv2
import sys
sys.path.append('build')
import numpy as np
import cv_uncc


im = cv2.imread('holes.jpg', cv2.IMREAD_GRAYSCALE)
imfilled = im.copy()
#cv_uncc.rgbd.fillHoles(imfilled)

cv_uncc.rgbd.RgbdImage(np.zeros((480,640,3),np.uint8), np.zeros((480,640),np.float32), 320, 240, 530);
#cv2.imshow("Original image", im)
#cv2.imshow("Python Module Function Example", imfilled)
#cv2.imshow("Python Module Class Example", imedge)
#cv2.waitKey(0)