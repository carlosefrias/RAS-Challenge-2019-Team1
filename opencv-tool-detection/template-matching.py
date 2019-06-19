import cv2
import numpy as np

import rospy, os

from std_msgs.msg import String
from matplotlib import pyplot as plt

def match(capture, template):
    img = capture.copy()
    width, height = template.shape[::-1]
    
    method = eval('cv2.TM_CCOEFF_NORMED')

    # Apply template Matching
    res = cv2.matchTemplate(img,template,method)
    _, confidence, min_loc, max_loc = cv2.minMaxLoc(res)

    top_left = max_loc
    bottom_right = (top_left[0] + width, top_left[1] + height)
    return top_left, bottom_right, confidence



img = cv2.imread('capture.png', 0)
img2 = img.copy()
template = cv2.imread('spanner.png', 0)

top_left, bottom_right, confidence = match(img, template)

print(top_left)
print(bottom_right)
print(confidence)

cv2.rectangle(img,top_left, bottom_right, 255, 2)

plt.imshow(img)
plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
plt.show()