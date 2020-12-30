#!/usr/bin/env python

import rospy
import time
import sys
from time import sleep
import rospy
from geometry_msgs.msg import Twist
import datetime


def main():   
    rospy.loginfo('test_cmd_vel - start!')
    rospy.init_node('test_cmd_vel', anonymous=True)
    cmd_pub = rospy.Publisher('/ppp/cmd_vel', Twist, queue_size=10)

    vel_msg = Twist()
    vel_msg.linear.x = 1.0/10.0 # 0.2 m/s=> 20 sek/m
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    rospy.loginfo('test_cmd_vel - publish val!'+ str(vel_msg.linear.x ))

    now = rospy.Time.now()
    rate = rospy.Rate(10)#!!!!!!!!!!!!!!!!!!!!!!!!
    rospy.loginfo(datetime.datetime.now().time())
    while rospy.Time.now() < now + rospy.Duration.from_sec(10):
            
            cmd_pub.publish(vel_msg)
            rate.sleep() 
    rospy.loginfo(datetime.datetime.now().time())       
    vel_msg.linear.x = 0
    cmd_pub.publish(vel_msg)
    rospy.spin()




if __name__ == '__main__':
    try:
        main()
    except:
        print("Unexpected error:", sys.exc_info()[0])
