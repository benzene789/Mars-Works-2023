import cv2
import imutils
import numpy as np
import os
import time

img = cv2.imread('output.png')
img = cv2.copyMakeBorder(img, 50, 50, 50, 50,cv2.BORDER_CONSTANT, (0, 0, 0))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
cv2.imshow('thresh',thresh)
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
c = max(cnts, key=cv2.contourArea)
mask = np.zeros(thresh.shape, dtype="uint8")
(x, y, w, h) = cv2.boundingRect(c)
img = img[y:y + h, x:x + w]

# cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)
minRect = mask.copy()
sub = mask.copy()
while cv2.countNonZero(sub) > 0:
    minRect = cv2.erode(minRect, None)
    sub = cv2.subtract(minRect, thresh)

cv2.imshow('frame',img)
time.sleep(10)
os._exit(0)
# cv2.imwrite('output.png',img)

