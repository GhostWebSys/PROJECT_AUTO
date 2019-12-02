# File : LaneDetect.py
# Date : 2019.11.23
# Developer : Ha Jae Chang (skymap87@gmail.com)

import rospy
import roslib

import sys, time
import numpy as np
import cv2

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

#from OpenCV_Utils import *
#from . import OpenCV_Utils
#import OpenCV_Utils

#roslib.load_manifest('my_package')
roslib.load_manifest('project_auto')
from project_auto.msg import LaneDetectMsg
g_hPubControl = None

g_iImageWidth = 0
g_iImageHeight = 0
g_iLane_Left_Max_x = 0
g_iLane_Left_Max_y = 0
g_iLane_Left_Min_x = 0
g_iLane_Left_Min_y = 0
g_iLane_Right_Max_x = 0
g_iLane_Right_Max_y = 0
g_iLane_Right_Min_x = 0
g_iLane_Right_Min_y = 0

def imageCopy(src):
    return np.copy(src)


def splitTwoSideLines(lines, slope_threshold = (5. * np.pi / 180.)):
    lefts = []
    rights = []
    for line in lines:
        x1 = line[0,0]
        y1 = line[0,1]
        x2 = line[0,2]
        y2 = line[0,3]
        if (x2-x1) == 0:
            continue
        slope = (float)(y2-y1)/(float)(x2-x1)
        if abs(slope) < slope_threshold:
            continue
        if slope <= 0:
            lefts.append([slope, x1, y1, x2, y2])
        else:
            rights.append([slope, x1, y1, x2, y2])
    return lefts, rights


def medianPoint(x):
    if len(x) == 0:
        return None
    else:
        xx = sorted(x)
        return xx[(int)(len(xx)/2)]


def interpolate(x1, y1, x2, y2, y):
    return int(float(y - y1) * float(x2-x1) / float(y2-y1) + x1)


def lineFitting(image, lines, color = (0,0,255), thickness = 3, slope_threshold = (5. * np.pi / 180.)):
    result = imageCopy(image)
    height = image.shape[0]
    lefts, rights = splitTwoSideLines(lines, slope_threshold)
    left = medianPoint(lefts)
    right = medianPoint(rights)
    min_y = int(height*0.6)
    max_y = height
    min_x_left = interpolate(left[1], left[2], left[3], left[4], min_y)
    max_x_left = interpolate(left[1], left[2], left[3], left[4], max_y)
    min_x_right = interpolate(right[1], right[2], right[3], right[4], min_y)
    max_x_right = interpolate(right[1], right[2], right[3], right[4], max_y)
    
    if(min_x_left < 0 or max_x_left < 0 or min_x_right < 0 or max_x_right < 0 ):
        print("Error - lane pos")
        return result

    cv2.line(result, (min_x_left, min_y), (max_x_left, max_y), color, thickness)
    cv2.line(result, (min_x_right, min_y), (max_x_right, max_y), color, thickness)

    global g_iLane_Left_Max_x, g_iLane_Left_Max_y, g_iLane_Left_Min_x, g_iLane_Left_Min_y
    global g_iLane_Right_Max_x, g_iLane_Right_Max_y, g_iLane_Right_Min_x, g_iLane_Right_Min_y
    g_iLane_Left_Min_x = min_x_left
    g_iLane_Left_Min_y = min_y
    g_iLane_Left_Max_x = max_x_left
    g_iLane_Left_Max_y = max_y
    g_iLane_Right_Min_x = min_x_right
    g_iLane_Right_Min_y = min_y
    g_iLane_Right_Max_x = max_x_right
    g_iLane_Right_Max_y = max_y
    return result


def makeBlackImage(image, color=False):
    height, width = image.shape[0], image.shape[1]
    if color is True:
        return np.zeros((height, width, 3), np.uint8)
    else:
        if len(image.shape) == 2:
            return np.zeros((height, width), np.uint8)
        else:
            return np.zeros((height, width, 3), np.uint8)


def fillPolyROI(image, points):
    if len(image.shape) == 2:
        channels = 1
    else:
        channels = image.shape[2]
    mask = makeBlackImage(image)
    ignore_mask_color = (255,) * channels
    cv2.fillPoly(mask, points, ignore_mask_color)
    return mask


def polyROI(image, points):
    mask = fillPolyROI(image, points)
    return cv2.bitwise_and(image, mask)


def houghLinesP(image, rho=1.0, theta=np.pi/180, threshold=100, minLineLength=10, maxLineGap=100):
    return cv2.HoughLinesP(image, rho, theta, threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)


def PublishLaneDetectionMsg(image):
    global g_iLane_Left_Max_x, g_iLane_Left_Max_y, g_iLane_Left_Min_x, g_iLane_Left_Min_y
    global g_iLane_Right_Max_x, g_iLane_Right_Max_y, g_iLane_Right_Min_x, g_iLane_Right_Min_y
    global g_hPubControl

    try:
       msg = LaneDetectMsg()
       #msg.msg_time = rospy.Time.now()
       msg.image_height, msg.image_width = image.shape[:2]

       msg.lane_left_min_x = g_iLane_Left_Min_x
       msg.lane_left_min_y = g_iLane_Left_Min_y
       msg.lane_left_max_x = g_iLane_Left_Max_x
       msg.lane_left_max_y = g_iLane_Left_Max_y
       msg.lane_right_min_x = g_iLane_Right_Min_x
       msg.lane_right_min_y = g_iLane_Right_Min_y
       msg.lane_right_max_x = g_iLane_Right_Max_x
       msg.lane_right_max_y = g_iLane_Right_Max_y

       # send msg
       g_hPubControl.publish(msg)
       #print("publish msg", g_iLane_Left_Min_x, g_iLane_Left_Min_y)
    except:
        print("Except - PublishLaneDetectionMsg()")

    return

def LaneDetection(data):
    try:
       np_arr = np.fromstring(data.data, np.uint8)
       image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
       #cv2.namedWindow("ori", cv2.WINDOW_NORMAL)
       #cv2.imshow("ori", image)

       hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       lower_white = np.array([0, 0, 100]) #100 ~ 160
       upper_white = np.array([179, 255, 255])
       mask = cv2.inRange(hsv, lower_white, upper_white)  
       #cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
       #cv2.imshow("mask", mask)
       image_edge = cv2.Canny(mask, 100, 200)
       #cv2.namedWindow("image_edge", cv2.WINDOW_NORMAL)
       #cv2.imshow("image_edge", image_edge)
    
       height, width = image.shape[:2]
       pt1 = (width*0.47, height*0.50)
       pt2 = (width*0.53, height*0.50)
       pt3 = (width*0.8, height*0.8)
       pt4 = (width*0.2, height*0.8)

       roi_corners = np.array([[pt1, pt2, pt3, pt4]], dtype=np.int32)
       image_roi = polyROI(image_edge, roi_corners)
       lines = houghLinesP(image_roi, 1, np.pi/180, 40)
       image_lane = lineFitting(image, lines, (0, 0, 255), 5, 5. * np.pi / 180.)
       cv2.namedWindow("lane_detect", cv2.WINDOW_NORMAL)
       cv2.imshow("lane_detect", image_lane)

       # send msg
       PublishLaneDetectionMsg(image)
    except:
       print("Except - imageProcessing()") 
    finally:
       cv2.waitKey(1)
    return


def main():
    # set node
    rospy.init_node('lane_detecter_node', anonymous = True)
    bridge = CvBridge()
      
    global g_hPubControl
    g_hPubControl = rospy.Publisher('lane_detecter', LaneDetectMsg, queue_size = 10)
    rospy.Subscriber("/front_cam/image/compressed", CompressedImage, LaneDetection, queue_size = 1)

    try:
        print("LaneDetect Start!\n")
        rospy.spin()  
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
