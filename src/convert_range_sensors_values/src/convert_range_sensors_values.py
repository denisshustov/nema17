#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
import rospy

# pub_laser = rospy.Publisher('/ppp/range_laser', LaserScan, queue_size=50)

pub_range1 = rospy.Publisher('/ppp/range_sensor1', Range, queue_size=10)
pub_range2 = rospy.Publisher('/ppp/range_sensor2', Range, queue_size=10)
pub_range3 = rospy.Publisher('/ppp/range_sensor3', Range, queue_size=10)

# def publishLaser(pub_laser, datas, frame_ids):
#     i = 0
#     for d in datas:
#         laser_msg = LaserScan()
#         laser_msg.header.stamp = rospy.Time.now()
#         laser_msg.header.frame_id = frame_ids[i]
#         laser_msg.angle_min = -1.57
#         laser_msg.angle_max = 1.57
#         #??????????????????
#         #https://www.theconstructsim.com/ros-qa-124-range-sensor-does-not-detect-obstacles-in-gazebo/
#         i+=1

def publishRange(range_publisher, data, frame_id):
    range_msg = Range()
    range_msg.header.stamp = rospy.Time.now()
    range_msg.header.frame_id = frame_id
    range_msg.radiation_type = 1
    range_msg.field_of_view = 0.34 #horizontal ~20, vertical ~13 -> radians 0.34
    range_msg.min_range = 0.02
    range_msg.max_range = 0.6
    range_msg.range = data/100.0
    range_publisher.publish(range_msg)



def callback(data):
    # publishLaser(pub_laser,data.data,["sonyc_1","sonyc_Mirror__1","sonyc__1__1"]) #right 

    publishRange(pub_range1,data.data[1],"sonyc_1") #right 
    publishRange(pub_range2,data.data[0],"sonyc_Mirror__1") # left
    publishRange(pub_range3,data.data[2],"sonyc__1__1") #forward

def main():
    global pub_range
    rospy.init_node('convert_range_sensors_values', anonymous=True)
    rospy.loginfo("convert_range_sensors_values initialization ok!")
    
    rospy.Subscriber("/rangeSonar1", Float32MultiArray, callback)
    rospy.spin()
    rospy.loginfo('convert_range_sensors_values - EXIT!!!')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass