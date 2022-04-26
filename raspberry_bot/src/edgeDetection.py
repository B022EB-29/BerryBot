#!/usr/bin/env python

import rospy
import cv2
import numpy as np

def rescaleFrame(frame, scale):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width,height)
    return cv2.resize(frame, dimensions, interpolation = cv2.INTER_AREA)

img = cv2.imread('/home/aril/Pictures/berryRowsLeikanger.jpeg')
img = rescaleFrame(img, 0.5)

blur = cv2.GaussianBlur(img, (7,7), cv2.BORDER_DEFAULT)
canny = cv2.Canny(blur, 50, 175)
dilated = cv2.dilate(canny, (7,7), iterations=5)
converterd_bitwiseNot = cv2.bitwise_not(dilated)

cv2.imshow('berryRows', img)
cv2.imshow('berryRows_blured', blur)
cv2.imshow('berryRows_canny', canny)
cv2.imshow('berryRows_filtered', converterd_bitwiseNot)

cv2.waitKey(0)
