# File : SensorCmdMux.py
# Date : 2019.11.26
# Developer : Ha Jae Chang (skymap87@gmail.com)
import rospy
import roslib
import time

# system
import queue
import threading

#roslib.load_manifest('my_package')
roslib.load_manifest('project_auto')
from project_auto.msg import LaneDetectMsg
from project_auto.msg import ControlMsg

# darknet ros msg
roslib.load_manifest('darknet_ros_msgs')
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox

# Ros Msg
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16MultiArray

# Image - OpenCV
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# System
g_qMsgQueue = None
g_hMsgQueuelock = None
g_hMsgWorkThread = None

g_qObjectCheckerQueue = None
g_hObjectCheckerQueuelock = None
g_hObjectCheckerWorkThread = None

g_bThreadExitFlag = False

# Lane Pos Info
g_bRecvLanePos = False
g_tiRecvLanePosTime = 0

g_iLane_Image_Wdith = 0
g_iLane_Image_Height = 0

g_iLane_Left_Max_x = 0
g_iLane_Left_Max_y = 0
g_iLane_Left_Min_x = 0
g_iLane_Left_Min_y = 0
g_iLane_Right_Max_x = 0
g_iLane_Right_Max_y = 0
g_iLane_Right_Min_x = 0
g_iLane_Right_Min_y = 0

g_iRoi_Lane_Left_Max_x = 0
g_iRoi_Lane_Left_Max_y = 0
g_iRoi_Lane_Left_Min_x = 0
g_iRoi_Lane_Left_Min_y = 0
g_iRoi_Lane_Right_Max_x = 0
g_iRoi_Lane_Right_Max_y = 0
g_iRoi_Lane_Right_Min_x = 0
g_iRoi_Lane_Right_Min_y = 0

#name = rospy.get_param("name", 3)

# Road Object
g_strRoadObjectClass = ["person", "bicycle",
                        "car", "motorbike", "bus", "train", "truck"]

# Ros
g_hPubControl = None
g_cvImage = None
g_ObjectBox = None

def PutQueueMsg(msg, queue, lock):
   bResult = False
   try:
      lock.acquire()
      #print("PutQueueMsg -lock")
      queue.put(msg)
      bResult = True
   except:
      print("Except - PutQueueMsg()")
   finally:
      lock.release()
   #print("PutQueueMsg - unlock")
   return bResult


def GetQueueMsg(queue, lock):
   msg = None

   try:
      lock.acquire()
      print("GetQueueMsg -lock")
      msg = queue.get()
   except:
      msg = None
     # print("Except - PopQueueMsg()")
   finally:
      lock.release()
   # print("GetQueueMsg - unlock")
   return msg


'''
def DrawImageObjectRectangle(image, point1=(x1, y1), point2=(x2, y2), color=(0,0,0), thickness=3):
   if image is not None:
      cv2.rectangle(image, point1, point2, (0,255,0), 5)
   return
'''

def ObjectBoundingBoxCallback(msg):
   #if(iEnableMsgPrint)
      #print(msg)
   try:
      global g_qObjectCheckerQueue, g_hObjectCheckerQueuelock, g_ObjectBox
      #PutQueueMsg(msg, g_qObjectCheckerQueue, g_hObjectCheckerQueuelock)
      g_ObjectBox = msg
   except:
      print("Except - ObjectBoundingBoxCallback()")
   return


def LaneDecterCallback(msg):  
   global g_bRecvLanePos, g_tiRecvLanePosTime
   global g_iLane_Image_Wdith, g_iLane_Image_Height
   global g_iLane_Left_Max_x, g_iLane_Left_Max_y, g_iLane_Left_Min_x, g_iLane_Left_Min_y
   global g_iLane_Right_Max_x, g_iLane_Right_Max_y, g_iLane_Right_Min_x, g_iLane_Right_Min_y

   try:
      #g_tiRecvLanePosTime = msg.stiTime
      #g_iLane_Image_Wdith = msg.image_width
      #g_iLane_Image_Height = image_height

      g_iLane_Left_Max_x = msg.lane_left_max_x
      g_iLane_Left_Max_y = msg.lane_left_max_y
      g_iLane_Left_Min_x = msg.lane_left_min_x
      g_iLane_Left_Min_y = msg.lane_left_min_y

      g_iLane_Right_Max_x = msg.lane_right_max_x
      g_iLane_Right_Max_y = msg.lane_right_max_y
      g_iLane_Right_Min_x = msg.lane_right_min_x
      g_iLane_Right_Min_y = msg.lane_right_min_y
      g_bRecvLanePos = True
      #print("Recv Lane Dectect1 ", g_iLane_Left_Max_x, g_iLane_Left_Max_y, g_iLane_Left_Min_x, g_iLane_Left_Min_y)
   except:
      print("Except - LaneDecterCallback()")
   #finally:
   return


def LidarDecterCallback(msg):
   try:
      print("")
   except:
      print("Except - LidarDecterCallback()")
   #finally:
   return


def ShowImageCallback(msg):
   try:
      np_arr = np.fromstring(msg.data, np.uint8)
      image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

      global g_cvImage
      g_cvImage = np.copy(image)
   except:
      print("Except - ShowImageCallback()")
   return


def MsgWorkThreadFunc():
   global g_qMsgQueue, g_hMsgQueuelock, g_bThreadExitFlag, g_hPubControl
   print("MsgWorkThreadFunc() Start.")

   while True:
      try:
         if(0 == g_qMsgQueue.qsize()):
            time.sleep(0.2)
         else: 
            g_hMsgQueuelock.acquire()
            msg = GetQueueMsg(g_qMsgQueue, g_hMsgQueuelock)
            g_hMsgQueuelock.alease()
            g_hPubControl.publish(msg)
      except:
         print("Except - WorkThreadFunc()")

      if(g_bThreadExitFlag):
         break

   print("MsgWorkThreadFunc() Exit.")
   return

def ShowMuxImage(cvImage):
   # Show Image
   if cvImage is not None:
      cv2.namedWindow("Sensor_Cmd_Mux", cv2.WINDOW_NORMAL)
      cv2.imshow("Sensor_Cmd_Mux", cvImage)
      cv2.waitKey(100)
   return

def ObjectCheckerThreadFunc():
   print("ObjectCheckerThreadFunc() Start.")
   global g_qMsgQueue, g_hMsgQueuelock
   global g_qObjectCheckerQueue, g_hObjectCheckerQueuelock, g_bThreadExitFlag
   global g_strRoadObjectClass, g_cvImage, g_ObjectBox
   cvImage = None

   iRoi_Lane_Left_Min_x = 0
   iRoi_Lane_Left_Min_y = 0
   iRoi_Lane_Left_Max_x = 0
   iRoi_Lane_Left_Max_y = 0
   iRoi_Lane_Right_Min_x = 0
   iRoi_Lane_Right_Min_y = 0
   iRoi_Lane_Right_Max_x = 0
   iRoi_Lane_Right_Max_y = 0

   fValue_Min_y = 0.2
   fValue_Max_x = 0.05
   iMinCenterPos = 0
   iMinCenterCap = 0
   iRoiMinY = 0
   iRoiMaxY = 0

   while True:
      print("\033[2J")
      print("\033[1;1H")  

      # Lane
      try:
         # Draw Lane
         if g_cvImage is not None:
            cvImage = np.copy(g_cvImage)

            # Calc Roi Lane      
            iMinCenterPos = int((g_iLane_Right_Min_x + g_iLane_Left_Min_x) / 2)
            iMinCenterCap = 0 #int(iMinCenterPos * 0.01)
            iRoiMinY = int(g_iLane_Left_Min_y - (g_iLane_Left_Min_y * fValue_Min_y))
            iRoiMaxY = g_iLane_Right_Max_y #same ly ry

            iRoi_Lane_Left_Min_x = iMinCenterPos - iMinCenterCap
            iRoi_Lane_Left_Min_y = iRoiMinY #int(g_iLane_Left_Min_y - (g_iLane_Left_Min_y * fValue_Min_y))
            iRoi_Lane_Left_Max_x = int(g_iLane_Left_Max_x - (g_iLane_Left_Max_x * fValue_Max_x))
            iRoi_Lane_Left_Max_y = g_iLane_Left_Max_y

            iRoi_Lane_Right_Min_x = iMinCenterPos + iMinCenterCap
            iRoi_Lane_Right_Min_y = iRoiMinY #int(g_iLane_Right_Min_y - (g_iLane_Right_Min_y * fValue_Min_y))
            iRoi_Lane_Right_Max_x = int(g_iLane_Right_Max_x + (g_iLane_Right_Max_x * fValue_Max_x))
            iRoi_Lane_Right_Max_y = g_iLane_Right_Max_y

            print("Lane 1 - ", iRoi_Lane_Left_Min_x, iRoi_Lane_Left_Min_y, iRoi_Lane_Left_Max_x, iRoi_Lane_Left_Max_y)
            print("Lane 2 - ", iRoi_Lane_Right_Min_x, iRoi_Lane_Right_Min_y, iRoi_Lane_Right_Max_x, iRoi_Lane_Right_Max_y)
            print("-----------------------------------------------------------")

            color = (0, 0, 255)
            thickness = 10
            cv2.line(cvImage, (g_iLane_Left_Min_x, g_iLane_Left_Min_y),(g_iLane_Left_Max_x, g_iLane_Left_Max_y), color, thickness)
            cv2.line(cvImage, (g_iLane_Right_Min_x, g_iLane_Right_Min_y),(g_iLane_Right_Max_x, g_iLane_Right_Max_y), color, thickness)

            color = (255, 0, 0)
            thickness = 10
            cv2.line(cvImage, (iRoi_Lane_Left_Min_x, iRoi_Lane_Left_Min_y), (iRoi_Lane_Left_Max_x, iRoi_Lane_Left_Max_y), color, thickness)
            cv2.line(cvImage, (iRoi_Lane_Right_Min_x, iRoi_Lane_Right_Min_y), (iRoi_Lane_Right_Max_x, iRoi_Lane_Right_Max_y), color, thickness)
      except:
         print("ObjectCheckerThreadFunc() - Draw lane except.")

      # Object
      try:
         '''
         if(0 == g_qObjectCheckerQueue.qsize()):
            # Show Image
            print("3-0")
            ShowMuxImage(cvImage)
            #time.sleep(0.2)
            print("g_qObjectCheckerQueue is 0.")
            continue
         '''

         # Get Msg
         #msg = GetQueueMsg(g_qObjectCheckerQueue, g_hObjectCheckerQueuelock)
         msg = g_ObjectBox

         if msg is None:
            ShowMuxImage(cvImage)
            print("g_ObjectBox is none.")
            continue
            
         bDectectObject = False

         # object filtering
         for objectData in msg.bounding_boxes:
            object_width = objectData.xmax - objectData.xmin
            object_height = objectData.ymax - objectData.ymin

            print("%-10s - pt1(x:%4d, y:%4d), pt2(x:%4d, y:%4d), size(w:%4d, h:%4d)" %
               (objectData.Class, objectData.xmin, objectData.ymin, objectData.xmax, objectData.ymax, object_width, object_height))
           
            if objectData.Class in g_strRoadObjectClass:
               color = (0, 255, 0)
               # check object size
               # object position in range?
               object_center_pos = objectData.xmin + (objectData.xmax - objectData.xmin)

               if(object_width > 5 and object_height > 5):   
                  if(((iRoiMinY <= objectData.ymax and objectData.ymax <= iRoiMaxY)
                     #and ((g_iLane_Left_Min_x <= objectData.xmin and objectData.xmin <= g_iLane_Right_Min_x))
                      #or (g_iLane_Left_Min_x <= objectData.xmax and objectData.xmax <= g_iLane_Right_Min_x))):
                     and ((g_iLane_Left_Min_x <= object_center_pos and object_center_pos <= g_iLane_Right_Min_x))
                      or (g_iLane_Left_Min_x <= object_center_pos and object_center_pos <= g_iLane_Right_Min_x))):
                  #if((iRoiMinY <= objectData.ymax and objectData.ymax <= iRoiMaxY)
                     #and (g_iLane_Left_Min_x <= object_center_pos and object_center_pos <= g_iLane_Right_Min_x)):

                     # Target Object - Red Box
                     color = (0, 0, 255)
                     bDectectObject = True
            else:
               color = (255, 255, 255)  # No Road Object Color

            if cvImage is not None:
               cv2.rectangle(cvImage, (objectData.xmin, objectData.ymin), (objectData.xmax, objectData.ymax), color, 5)
   
         if(bDectectObject):
            cmdMsg = ControlMsg()
            cmdMsg.send_time = rospy.Time.now()
            cmdMsg.msg_type = 0
            cmdMsg.value_1 = 0
            cmdMsg.value_2 = 0.0

            global g_hPubControl
            
            if g_hPubControl is not None:
               g_hPubControl.publish(cmdMsg)
            """
            if g_qMsgQueue is not None:
               g_hMsgQueuelock.acquire()
               #PutQueueMsg(cmdMsg, g_qMsgQueue, g_hMsgQueuelock)
               #g_qMsgQueue.put(msg)
               
               g_hMsgQueuelock.release()
            """
            
      except:
         print("Except - ObjectCheckerThreadFunc()")

      # Show Image
      ShowMuxImage(cvImage)
      g_ObjectBox = None

      time.sleep(0.01)
      if(g_bThreadExitFlag):
         break

   print("ObjectCheckerThreadFunc() Exit.")
   return


def Init():
   # Set Node
   global g_hPubControl
   g_hPubControl = rospy.Publisher('car_control_msg', ControlMsg, queue_size = 10)
   rospy.init_node('sensor_cmd_mux', anonymous = True)
   
   # System
   global g_bThreadExitFlag

   # Create cmd msg queue, thread
   global g_qMsgQueue, g_hMsgWorkThread, g_hQueuelock
   g_qMsgQueue = queue.Queue(100)
   g_hQueuelock = threading.Lock()
   g_hMsgWorkThread = threading.Thread(target = MsgWorkThreadFunc)
   g_hMsgWorkThread.setDaemon(True)
   g_hMsgWorkThread.start()

   # Create object checker queue, thread
   global g_qObjectCheckerQueue, g_hObjectCheckerWorkThread, g_hObjectCheckerQueuelock
   g_qObjectCheckerQueue = queue.Queue(100)
   #g_hObjectCheckerQueuelock = threading.Lock()
   g_hObjectCheckerWorkThread = threading.Thread(target = ObjectCheckerThreadFunc)
   g_hObjectCheckerWorkThread.setDaemon(True)
   g_hObjectCheckerWorkThread.start()
   
   # recv topice
   rospy.Subscriber("/front_cam/image/compressed", CompressedImage, ShowImageCallback, queue_size = 1)  
   rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, ObjectBoundingBoxCallback, queue_size = 1)
   rospy.Subscriber('lane_detecter', LaneDetectMsg, LaneDecterCallback, queue_size = 1)
   rospy.Subscriber('filter', Int16MultiArray, LidarDecterCallback, queue_size = 1)

   rospy.spin()
   print("Ros exit.")
   return
   

def Close():
   # Thread exit
   global g_hMsgWorkThread, g_bThreadExitFlag
   g_bThreadExitFlag = True

   # Etc
   cv2.destroyAllWindows()
   return

def main():
   print("SensorCmdMux Start.")
   Init()
   Close()

if __name__ == '__main__':
   main()
