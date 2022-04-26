#!/usr/bin/env python

import numpy as np
import cv2
yellow = np.uint8([[[47,239,99]]])
hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
print(hsv_yellow)
