import cv2
import numpy as np

import rospy, os

from std_msgs.msg import String
from matplotlib import pyplot as plt

def rotate_image(mat, angle):
    """
    Rotates an image (angle in degrees) and expands image to avoid cropping
    """

    height, width = mat.shape[:2] # image shape has 3 dimensions
    image_center = (width/2, height/2) # getRotationMatrix2D needs coordinates in reverse order (width, height) compared to shape

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

    # rotation calculates the cos and sin, taking absolutes of those.
    abs_cos = abs(rotation_mat[0,0]) 
    abs_sin = abs(rotation_mat[0,1])

    # find the new width and height bounds
    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)

    # subtract old image center (bringing image back to origo) and adding the new image center coordinates
    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]

    # rotate image with the new bounds and translated rotation matrix
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h), borderValue=(255,255,255))
    return rotated_mat


def ger_confidence(capture, template, angle, method):
        templateRotated = rotate_image(template, angle)
        # Apply template Matching
        res = cv2.matchTemplate(img, templateRotated, method)
        _, confidence, _, _ = cv2.minMaxLoc(res)
        return confidence

def match(capture, template):
        method = eval('cv2.TM_CCOEFF_NORMED')
        angle_range = range(0, 360, 1)
        values = [ger_confidence(capture, template, angle, method) for angle in angle_range]
        max_idx = values.index(max(values))
        best_angle = angle_range[max_idx]
        print("best rotation angle: " + str(best_angle))
        templateRotated = rotate_image(template, best_angle)
        cv2.imwrite("rotated_template.png", templateRotated)
        # Apply template Matching
        res = cv2.matchTemplate(img, templateRotated, method)
        _, confidence, _, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        width, height = templateRotated.shape[::-1]
        bottom_right = (top_left[0] + width, top_left[1] + height)
        return top_left, bottom_right, confidence

img = cv2.imread('capture3.png', 0)
img2 = img.copy()
template = cv2.imread('hammer.png', 0)

top_left, bottom_right, confidence = match(img, template)
print("top_left point:")
print(top_left)
print("bottom right point:")
print(bottom_right)
print("confidence level:")
print(confidence)

cv2.rectangle(img,top_left, bottom_right, 255, 2)

plt.imshow(img)
plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
plt.show()