#! /usr/bin/env python
#[WARN] [1574752851.677670]: cnt: 299 , degrees: 35.991 , r : 6.983
#위 같이 출력이 나오는데 각이 처음에 음수로 시작해 -0~-90[1사분면] +89~0[4사분면]
# -0 ~ -90[3사분면] +89 ~ +0 [2사분면] 90도마다 부호가 변합니다~!
# | -> / ->  - -> \ -> | -> / -> - -> \ -> |   
#
#

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time
import math
from std_msgs.msg import Int16MultiArray
pub = rospy.Publisher('filter', Int16MultiArray, queue_size=1)
def callback_pointcloud(data):
    start = time.time()
    
    mat = Int16MultiArray()
    mat.data=[]


#    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y"), skip_nans=True)
#    time.sleep(0.0000001)
#    print(type(gen))
    ch =1
    cnt =0
    flag=0
            #ch   2    3    4     5     6    7    8    9    10    11    12      13    14 
    cutoffR=[0,0,4.1, 4.5, 4.8, 5.0,   5.6, 6.6, 7.4, 8.7, 10.0, 12.8, 17.1,  24.3, 40,0,0,0,0,0]
    cutoffS=[0,0, 21,  19,  18,   16,   14,  13,  11,   9,    8,    6,    4,     3,    3]
    for p in gen:
       degrees = math.degrees(math.atan(p[1]/p[0]))
       if abs(degrees)<0.0001: #degrees == 0
           if flag!=0:
               cnt = cnt + 1
               if cnt ==2:
                   ch = ch+1
                   cnt = 0
           else:
               flag = flag + 1


       if ch > 4 and ch < 15:
           if cnt == 0 and degrees < 0 and -int(round(degrees))<=cutoffS[ch]:
                r=math.sqrt(p[0]*p[0]+p[1]*p[1])
                if r<cutoffR[ch]: 
                    mat.data=[ch,int(degrees),int(r)]
                    pub.publish(mat)
                    rospy.logwarn("ch: %d , degrees: %f , r : %.3f, %d" %(ch,degrees ,r, cnt))
                    break
           elif cnt == 1 and degrees > 0 and int(round(degrees))<=cutoffS[ch]:
                r=math.sqrt(p[0]*p[0]+p[1]*p[1])
                if r<cutoffR[ch]: 
                    mat.data=[ch,int(degrees),int(r)]
                    pub.publish(mat)
                    rospy.logwarn("ch: %d , degrees: %f , r : %.3f, %d" %(ch,degrees ,r, cnt))
                    break
    print(time.time()-start)


#       rospy.logwarn("ch: %d , degrees: %f , r : %.3f, %d" %(ch,degrees ,r, cnt))

'''
       if ch > 1 and r<cutoffR[ch]: 
            if cnt == 0 and degrees < 0 and -int(round(degrees))<=cutoffS[ch]:
               #rospy.logwarn("R ch: %d , degrees: %f , r : %.3f, %d" %(ch,degrees ,r, cnt))
               mat.data=[ch,int(degrees),int(r)]
               pub.publish(mat)
              
            elif cnt == 1 and degrees > 0 and int(round(degrees))<=cutoffS[ch]:
               #rospy.logwarn("L ch: %d , degrees: %f , r : %.3f, %d" %(ch,degrees ,r, cnt))
               mat.data=[ch,int(degrees),int(r)]
               pub.publish(mat)
'''

def main():
    rospy.init_node('lidar_filter', anonymous=True)
    rospy.Subscriber('/points_raw', PointCloud2, callback_pointcloud,queue_size=1,tcp_nodelay=1)
    rospy.spin()

if __name__ == "__main__":
    main()

