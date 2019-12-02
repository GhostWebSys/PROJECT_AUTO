# File : joy_control_lgsvlsim.py
# Date : 2019.11.23
# Developer : 하재창 (Ha Jae Chang / skymap87@gmail.com)
# Environment : Ubuntu 18.04, ROS melodic, Python3, joystick(Logietch F710 - D mode), xdotool

# This file is use xdotool package.
# ROS (send joystick msg) -> this file (joystick msg receive and convert. and send key msg.) -> LG SVL Simulator

# How to use
# 1. Install xdotool package.
#   >> $ sudo apt-get install xdotool
#   >> if it is installed, skip this step. 
# 2. ROS(melodic) Start
#  2-1. Ros start.
#    >> $ roscore
#  2-2. joystick change chmod 777 (js[x] : [x] is joystick device number)
#    >> $ sudo chmod 777 /dev/input/js0
#  2-3. joy_node start.
#    >> rosrun joy joy_node
# 3. LG SVL simulator Start.
# 4. run this file and control joystick.
#   >> Python3 joy_control_lgsvlsim.py

import rospy
from sensor_msgs.msg import Joy
import os
import sys

iChangeCamMode = 0

def callback(msg):
    print(msg)

    strCommand = 'window_id=`xdotool search --name "LGSVL Simulator"` && xdotool windowactivate $window_id '
    strInputKey = ''

    if((0.0 == msg.axes[0] and 0.0 == msg.axes[1]) or (0.0 == msg.axes[5] and 0.0 == msg.axes[4])):
       strInputKey += "keyup Up Down Left Right "

    if(0.0 < msg.axes[1] or 0.0 < msg.axes[5]):
       strInputKey += "keydown Up "
    elif(0.0 > msg.axes[1] or 0.0 > msg.axes[5]):
       strInputKey += "keydown Down "

    if(0.0 < msg.axes[0]  or 0.0 < msg.axes[4]):
       strInputKey += "keydown Left "
    elif(0.0 > msg.axes[0] or 0.0 > msg.axes[4]):
       strInputKey += "keydown Right "

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
    if(1 == msg.buttons[3]):
       strInputKey += "key Page_Up "

    '''
    # another joystick type - check your joystick model
    if(1 == msg.buttons[0]):
       strInputKey += "key Page_Up "
    if(1 == msg.buttons[1]):
       strInputKey += "key Shift_R "
    if(1 == msg.buttons[2]):
       strInputKey += "key Page_Down "
    if(1 == msg.buttons[3]):
       strInputKey += "key m "
    '''

    if(1 == msg.buttons[4]):
       strInputKey += "key comma "
    if(1 == msg.buttons[5]):
       strInputKey += "key period "

    if(1 == msg.buttons[6]):
       strInputKey += "key n "
    if(1 == msg.buttons[7]):
       strInputKey += "key p "

    if(1 == msg.buttons[8]):
       global iChangeCamMode
       iChangeCamMode += 1
       if(iChangeCamMode % 2):
         strInputKey += "key 1 "
       else:
         strInputKey += "key asciitilde "

    if(1 == msg.buttons[9]):
       strInputKey += "key F12 "
    if(1 == msg.buttons[10]):
       strInputKey += "key h "
    if(1 == msg.buttons[11]):
       strInputKey += "key alt+Print "

    if(strInputKey != ''):
       strCommand += strInputKey
       os.system(strCommand)
       print(strCommand)

def listener():
    rospy.init_node('joy_convert_node', anonymous = True)
    rospy.Subscriber("joy", Joy, callback, queue_size=1)
    print("joystick control start!\n")
    rospy.spin()

if __name__ == '__main__':
    listener()
