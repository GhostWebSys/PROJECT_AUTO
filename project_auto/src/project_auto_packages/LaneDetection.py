# File : SensorCmdMux.py
# Date : 2019.11.28
# Developer : 하재창 (Ha Jae Chang / skymap87@gmail.com) - ROS Topic Publisher & Subscriber
# Developer : 김상민 (Kim Sang Min / sm.kim@kakao.com) - Lane Detection using OpenCV

from __future__ import print_function
import sys, time
import numpy as np
import cv2
import pickle
import glob
import matplotlib.pyplot as plt
from ipywidgets import interact, interactive, fixed
from moviepy.editor import VideoFileClip
from IPython.display import HTML

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# ROS TOPIC Publisher
roslib.load_manifest('project_auto')
from project_auto.msg import LaneDetectMsg
g_hPubControl = None

def unwarp(img, src, dst):
    h,w = img.shape[:2]
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(img, M, (w,h), flags=cv2.INTER_LINEAR)
    return warped, M, Minv

def splitTwoSideLines(lines, slope_threshold = (5. * np.pi / 180.)):
    lefts = []
    rights = []
    if lines is None:
        return None, None

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
    if x is None:
        return None
    if len(x) == 0:
        return None
    else:
        xx = sorted(x)
        return xx[(int)(len(xx)/2)]


def interpolate(x1, y1, x2, y2, y):
    return int(float(y - y1) * float(x2 - x1) / float(y2 - y1) + x1)


def lineFitting(image, lines, color=(0,0,255), thickness = 3, slope_threshold = (5. * np.pi / 180.)):
    result = np.zeros_like(image)
    tmp = []
    #result = np.copy(image)
    height = image.shape[0]
    lefts, rights = splitTwoSideLines(lines, slope_threshold)
    min_y = 590
    max_y = height
    left = medianPoint(lefts)
    right = medianPoint(rights)
    if left is not None:
        min_x_left = interpolate(left[1], left[2], left[3], left[4], min_y)
        max_x_left = interpolate(left[1], left[2], left[3], left[4], max_y)
        tmp.append(min_x_left) # left upper point
        tmp.append(max_x_left) # left lower point
        cv2.line(result, (min_x_left, min_y), (max_x_left, max_y), color, thickness)
    else:
        tmp.append(0)
        tmp.append(0)
    if right is not None:
        min_x_right = interpolate(right[1], right[2], right[3], right[4], min_y)
        max_x_right = interpolate(right[1], right[2], right[3], right[4], max_y)
        tmp.append(min_x_right) #right upper point
        tmp.append(max_x_right) #right lower point
        cv2.line(result, (min_x_right, min_y), (max_x_right, max_y), color, thickness)
    else:
        tmp.append(0)
        tmp.append(0)
    tmp.append(min_y)
    tmp.append(max_y)
    return result, tmp


def sendMessage(w, h, yellow_position, white_position):
    global g_hPubControl

    msg = LaneDetectMsg()
    msg.image_width = w
    msg.image_height = h

    msg.lane_left_max_x = yellow_position[1]
    msg.lane_left_max_y = 850
    msg.lane_left_min_x = yellow_position[0]
    msg.lane_left_min_y = 590

    msg.lane_right_max_x = white_position[3]
    msg.lane_right_max_y = 850
    msg.lane_right_min_x = white_position[2]
    msg.lane_right_min_y = 590

    g_hPubControl.publish(msg)
    return


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/front_cam/image/compressed", CompressedImage, self.callback, queue_size = 1)


    def callback(self, data):
        try:
            np_arr = np.fromstring(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)


        cv2.namedWindow("cv_image", cv2.WINDOW_NORMAL)
        image = np.copy(cv_image)
        image = image[:850, :1920]

        h,w = image.shape[:2]
        src = np.float32([(850,590), (1080,590), (450,850), (1470,850)])
        dst = np.float32([(450,0), (w-450,0), (450,h), (w-450,h)])

        # ROI Setting
        newImage = np.copy(image)
        vertices = np.array([[(850, 590), (1080,590), (1470,850), (450, 850)]])
        mask = np.zeros_like(newImage)
        cv2.fillPoly(mask, vertices, (255,255,255))
        roiImage = cv2.bitwise_and(newImage, mask)
        #cv2.imshow("ROI IAMGE", roiImage)
        #newImage, M, Minv = unwarp(image, src, dst) # if you set bird-eye-view, remove upper code and use it

        # Find Yellow Lines
        frame = cv2.GaussianBlur(roiImage, (5, 5), 0)
        hsv = cv2.cvtColor(roiImage, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([0, 100, 110])
        yellow_upper = np.array([180, 255, 255])
        yellowMask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        edges = cv2.Canny(yellowMask, 75, 150)
        yellow_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap = 50000)

        # Find White Lines
        mark = np.copy(roiImage)
        blue_threshold = 85
        green_threshold = 70
        red_threshold = 70
        bgr_threshold = [blue_threshold, green_threshold, red_threshold]
        thresholds = (roiImage[:,:,0] < bgr_threshold[0]) | (roiImage[:,:,1] < bgr_threshold[1]) | (roiImage[:,:,2] < bgr_threshold[2])
        mark[thresholds]= [0,0,0]
        white_edges = cv2.Canny(mark, 35, 35)
        white_lines = cv2.HoughLinesP(white_edges, 1, np.pi / 180, 50, maxLineGap = 50000)

        image_yellow_lane, yellow_position = lineFitting(roiImage, yellow_lines, (0, 0, 255), 5, 5. * np.pi / 180.)
        image_white_lane, white_position = lineFitting(roiImage, white_lines, (0, 255, 0), 5, 5. * np.pi / 180.)

        imageLane = cv2.add(image_yellow_lane, image_white_lane)
        #newWarp = cv2.warpPerspective(imageLane, Minv, (w,h))
        finished_image = cv2.addWeighted(image, 1, imageLane, 1, 0)
        cv2.imshow("cv_image", finished_image)

        # Send to ros
        sendMessage(h, w, yellow_position, white_position)
        cv2.waitKey(3)


def main(args):
    ic = image_converter()
    rospy.init_node('lane_detecter_node', anonymous = True)

    #cv2.resizeWindow("cv_image", 640, 480);

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

def Init():
    global g_hPubControl
    g_hPubControl = rospy.Publisher('lane_detecter', LaneDetectMsg, queue_size = 10)

if __name__ == '__main__':
    Init()
    main(sys.argv)
