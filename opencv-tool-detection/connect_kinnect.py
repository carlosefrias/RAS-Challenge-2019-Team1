#!/usr/bin/env python
from __future__ import print_function

import roslib
import template_matching
roslib.load_manifest('matching_tool')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.callback)

    self.picture_taken = False

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    if(not self.picture_taken):
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
      cv2.imwrite('capturex.png', gray)
      _, offset, angle = template_matching.get_point(gray, "spanner")
      self.picture_taken = True

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

  # img = cv2.imread('capture5.png', 0)

  # template_matching.get_point(img, "hammer")


if __name__ == '__main__':
    main(sys.argv)