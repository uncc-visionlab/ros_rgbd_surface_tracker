#!/usr/bin/env python
import cv2
import sys
sys.path.append('build')
import cv_uncc


im = cv2.imread('holes.jpg', cv2.IMREAD_GRAYSCALE)
imfilled = im.copy()
cv_uncc.rgbd.fillHoles(imfilled)

filters = cv_uncc.rgbd.Filters() 
imedge = filters.edge(im)


cv2.imshow("Original image", im)
cv2.imshow("Python Module Function Example", imfilled)
cv2.imshow("Python Module Class Example", imedge)
cv2.waitKey(0)