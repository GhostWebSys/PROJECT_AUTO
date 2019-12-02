# File : SimControl.py
# Date : 2019.11.23
# Developer : Ha Jae Chang (skymap87@gmail.com)
import rospy
import roslib
import os
import sys

# msg
from sensor_msgs.msg import Joy

roslib.load_manifest('project_auto')
from project_auto.msg import ControlMsg

g_iChangeCamMode = 0
g_bEmergencyStopFlag = False
g_bIgnoreStopMsg = False

def DisplayStatus():
   print("\033[2J")
   print("\033[1;1H")
   strStatus = "Status : "
   if(g_bEmergencyStopFlag):
      strStatus += "Emergency Stop"
   else:
      strStatus += "Nomal"

   strStatus += "\nIgnore Stop Msg : "

   if(g_bIgnoreStopMsg):
      strStatus += "ON"
   else:
      strStatus += "OFF"

   print(strStatus)
   return

def GetDefaultSimControlString():
   return 'window_id=`xdotool search --name "LGSVL Simulator"` && xdotool windowactivate $window_id '

def JoystickCallback(msg):
    global g_bEmergencyStopFlag, g_bIgnoreStopMsg
    DisplayStatus()
    print(msg, "\n")

    strCommand = 'window_id=`xdotool search --name "LGSVL Simulator"` && xdotool windowactivate $window_id '
    strInputKey = ''

    if(False == g_bEmergencyStopFlag):
       if((0.0 == msg.axes[0] and 0.0 == msg.axes[1]) or (0.0 == msg.axes[5] and 0.0 == msg.axes[4])):
          strInputKey += "keyup Up Down Left Right "

       if(0.0 < msg.axes[1] or 0.0 < msg.axes[5]):
          strInputKey += "keydown Up "
       elif(0.0 > msg.axes[1] or 0.0 > msg.axes[5]):
          strInputKey += "keydown Down "

       if(0.0 < msg.axes[0] or 0.0 < msg.axes[4]):
          strInputKey += "keydown Left "
       elif(0.0 > msg.axes[0] or 0.0 > msg.axes[4]):
          strInputKey += "keydown Right "
    else:
       print("Emergency Stop Activate! - You can't cotrol car. Car control msg is ignore.")

    if(0.0 == msg.axes[2] and 0.0 == msg.axes[3]):
       strInputKey += "keyup w s a d "
    if(0.0 < msg.axes[2]):
       strInputKey += "keydown a "
    elif(0.0 > msg.axes[2]):
       strInputKey += "keydown d "
    if(0.0 < msg.axes[3]):
       strInputKey += "keydown w "
    elif(0.0 > msg.axes[3]):
       strInputKey += "keydown s "

    # button
    if(1 == msg.buttons[0]):
       strInputKey += "key m "
    if(1 == msg.buttons[1]):
       strInputKey += "key Page_Down "
    if(1 == msg.buttons[2]):
       strInputKey += "key Shift_R "
       if(g_bEmergencyStopFlag):
           g_bEmergencyStopFlag = False
           strInputKey += "m keyup Up Down Left Right "
           print("Emergency Stop Disarmed!")
    if(1 == msg.buttons[3]):
       strInputKey += "key Page_Up "

    if(1 == msg.buttons[4]):
       strInputKey += "key comma "
    if(1 == msg.buttons[5]):
       strInputKey += "key period "

    if(1 == msg.buttons[6]):
       strInputKey += "key n "
    if(1 == msg.buttons[7]):
       strInputKey += "key p "

    if(1 == msg.buttons[8]):
       global g_iChangeCamMode
       g_iChangeCamMode += 1
       if(g_iChangeCamMode % 2):
         strInputKey += "key 1 "
       else:
         strInputKey += "key asciitilde "

    if(1 == msg.buttons[9]):
       strInputKey += "key F12 "
    if(1 == msg.buttons[10]):
       #strInputKey += "key h "
       g_bIgnoreStopMsg = not g_bIgnoreStopMsg
       if(g_bIgnoreStopMsg):
          print("Enable ignore stop msg.")
       else:
          print("Enable handing stop msg.")

    if(1 == msg.buttons[11]):
       strInputKey += "key alt+Print "

    if(strInputKey != ''):
       strCommand += strInputKey
       os.system(strCommand)
       #print(strCommand)
    return

def CarEmergencyStop(msg):
    global g_bEmergencyStopFlag, g_bIgnoreStopMsg
    DisplayStatus()

    if(g_bIgnoreStopMsg):
       return

    if(False == g_bEmergencyStopFlag):
       g_bEmergencyStopFlag = True
       strCommand = 'window_id=`xdotool search --name "LGSVL Simulator"` && xdotool windowactivate $window_id keyup Up Down Left Right key Shift_R m keydown Down'
       os.system(strCommand)
       #print(strCommand)
       print("Emergency Stop Activate!")
    else:
       print("")
    return

def CarMove(msg):
    if(0 != msg.msg_type):
       return

    DisplayStatus()
    strCommand = GetDefaultSimControlString()
    strInputKey = ''

    if((0 == msg.value_1 and 0 == msg.value_1)
       or (0.0 == msg.value_2 and 0.0 == msg.value_2)):
       strInputKey += "keyup Up Down Left Right "

    # go, back
    if(0 < msg.value_1):
       strInputKey += "keydown Up "
    elif(0 > msg.value_1):
       strInputKey += "keydown Down "

    # left, right
    if(0.0 < msg.value_2):
       strInputKey += "keydown Left "
    elif(0.0 > msg.value_2):
       strInputKey += "keydown Right "

    if(strInputKey != ''):
       strCommand += strInputKey
       os.system(strCommand)
       print(strCommand)
    strCommand = GetDefaultSimControlString()
    strInputKey = ''

    strInputKey = {0: "key m ",
                   1: "key Page_Down ",
                   2: "key Shift_R ",
                   3: "key Page_Up ",
                   4: "key comma ",
                   5: "key period ",
                   6: "key n ",
                   7: "key p ",
                   8: "key 1 ",
                   9: "key F12 ",
                   10: "key h ",
                   11: "key alt+Print "}.get(msg.value_1)

    if strInputKey is None:
       print("Error - Unknow key msg! (value_1 : %d)" % (msg.value_1))
       return

    if(strInputKey != ''):
       strCommand += strInputKey
       os.system(strCommand)
       print(strCommand)

def CarEtcControl(msg):
   DisplayStatus()
   return

def CarControlMsgCallback(msg):
    functions = {0: CarEmergencyStop, 
                 1: CarMove,
                 2: CarEtcControl}.get(msg.msg_type)

    if functions is None:
       print("Error - This msg type is not suport. (msg_type : %d)" %
             (msg.msg_type))
       return

    functions(msg)


def listener():
    # set node
    rospy.init_node('sim_control', anonymous=True)

    # recv topic
    rospy.Subscriber("joy", Joy, JoystickCallback, queue_size = 1)
    rospy.Subscriber('car_control_msg', ControlMsg, CarControlMsgCallback)
    print("SimControl Start!\n")
    rospy.spin()

def main():
    listener()

if __name__ == '__main__':
    main()
