import cv2
import numpy as np
import time
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

    # rotate image with the new bounds and translated rotation matrix and fills the corners with white color
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h), borderValue=(255,255,255))
    return rotated_mat

def get_confidence(capture, template, angle, method):
        """
        Gets the template matching confidence of the image template against the captured image 
        with a rotation of angle degrees around the template center
        """
        templateRotated = rotate_image(template, angle)
        # cv2.imwrite("rotated_template_" + str(angle) + ".png", templateRotated)
        # Apply template Matching
        res = cv2.matchTemplate(img, templateRotated, method)
        _, confidence, _, _ = cv2.minMaxLoc(res)
        return confidence

def match(capture, template):
        """
        Runs the template matching for all whole angles between 0 and 360
        and determines which is the one with the best score
        Returns top left and bottom right coordinates of the bounding box containing the item,
        the matching score and the rotation angle in degrees
        """
        method = eval('cv2.TM_CCOEFF_NORMED')
        angle_range = range(-180, 180, 1)
        values = [get_confidence(capture, template, angle, method) for angle in angle_range]
        max_idx = values.index(max(values))
        best_angle = angle_range[max_idx]
        templateRotated = rotate_image(template, best_angle)
        # cv2.imwrite("rotated_template.png", templateRotated)
        # Apply template Matching
        res = cv2.matchTemplate(img, templateRotated, method)
        _, confidence, _, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        width, height = templateRotated.shape[::-1]
        bottom_right = (top_left[0] + width, top_left[1] + height)
        return top_left, bottom_right, confidence, best_angle

def calc_offset(top_left, bottom_right, img, template, tool_length):
        """
        Calculates the horizontal and vertical offset in millimeters from the
        center of the captured image to the center of the tool
        """
        tl = np.asarray(top_left)
        br = np.asarray(bottom_right)
        centroid = (tl + br) / 2
        width_pixels, height_pixels = img.shape[::-1]
        image_center = np.asarray([width_pixels / 2, height_pixels / 2])
        offset = centroid - image_center
        _, height_pixels_tool = template.shape[::-1]
        offset *= tool_length / height_pixels_tool
        return centroid, offset

def calc_picking_point(centroid, dist, angle, template, tool_length):
        x_offset = dist * np.cos(np.radians(angle))
        print(x_offset)
        y_offset = dist * np.sin(np.radians(angle))
        print(y_offset)
        offset = [x_offset, y_offset]
        point = centroid + offset
        print(point)
        _, height_pixels_tool = template.shape[::-1]
        point *= height_pixels_tool / tool_length
        return point

def get_point(img, tool_name):
        indx = 0
        if (tool_name == "hammer"):
                indx = 0
        elif (tool_name == "spanner"):
                indx = 1
        elif (tool_name == "screw-driver"):
                indx = 2
        else:
                ind = -1

        tools = ["hammer", "spanner", "screw-driver"]
        lengths = [300, 240, 290]
        gripping_distance = [110, 0, 100]

        tool = tools[indx]
        length = lengths[indx]
        dist = gripping_distance[indx]
        template = cv2.imread(tool + '.png', 0)
        top_left, bottom_right, confidence, angle = match(img, template)

        centroid, offset = calc_offset(top_left, bottom_right, img, template, length)
        return centroid, offset, angle
        
### Main execution ##########################################
# img = cv2.imread('capture5.png', 0)
# img2 = img.copy()

# tools = ["hammer", "spanner", "screw-driver"]
# lengths = [300, 240, 290]
# gripping_distance = [110, 0, 100]

# tool = tools[2]
# length = lengths[2]
# dist = gripping_distance[2]

# template = cv2.imread(tool + '.png', 0)

# start = time.time()
# top_left, bottom_right, confidence, angle = match(img, template)

# centroid, offset = calc_offset(top_left, bottom_right, img, template, length)
# # point  = calc_picking_point(centroid, dist, angle, template, length)
# print("elapsed time:")
# print(time.time() - start)

# print("top_left point:")
# print(top_left)
# print("bottom right point:")
# print(bottom_right)
# print("confidence level:")
# print(confidence)
# print("rotation angle")
# print(angle)

# print("centroid:")
# print(centroid)
# print("offset from the center in milimeters: ")
# print(offset)

# # print("point")
# # print(point)

# ###### Ploting results ############
# centroid = (centroid[0], centroid[1])
# cv2.rectangle(img,top_left, bottom_right, 0, 2)
# cv2.circle(img, centroid, 5, 0,2) 
# # point = (int(point[0]), int(point[1]))
# # cv2.circle(img, point, 5, 255,2) 
# plt.imshow(img,cmap = 'gray')
# plt.title('Detected ' + tool), plt.xticks([]), plt.yticks([])
# plt.show()
#### End of program ###############