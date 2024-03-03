# A vision system will be shared between controllers and the main program. As its a shared vision system, 
# it is hard to synchronize if each indiv controller have a vision node running. Thus the vision node will be
# running on the main program and the controller will be subscribing to the vision node to get the desired data, via rosbridge.

# Vision data will also be processed here, and the processed data will be published to the controller via rosbridge. 

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_web_republisher
import geometry_msgs.msg

