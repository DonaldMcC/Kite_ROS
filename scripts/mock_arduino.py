#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from std_msgs.msg import String, Int16
motorvalue = 0

def listen_motormsg():
    rospy.Subscriber('motor_msg', Int16, callback)

def callback(data):
    global motorvalue
    print ('got data', data)
    motorvalue = data
    return



def kiteangle(barangle):
    pub = rospy.Publisher('kiteangle', Int16, queue_size=3)
    rospy.init_node('mock_arduino', anonymous=False)
    rate = rospy.Rate(5)  # 5hz
    while not rospy.is_shutdown():
        print('motorv', motorvalue)
        rospy.loginfo(barangle)
        pub.publish(barangle)
        rate.sleep()
        barangle += 1
        if barangle >= 1005:
            barangle = 674
    return


if __name__ == '__main__':
    try:
        kiteangle(100)
    except rospy.ROSInterruptException:
        pass