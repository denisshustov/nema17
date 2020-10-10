#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
import rospy

pub_range1 = rospy.Publisher('/ppp/range1', Range, queue_size=10)
pub_range2 = rospy.Publisher('/ppp/range2', Range, queue_size=10)
pub_range3 = rospy.Publisher('/ppp/range3', Range, queue_size=10)

def publishRange(range_pub, data, frame_id):
    range_pub = Range()
    range_pub.header.stamp = rospy.Time.now()
    range_pub.header.frame_id = frame_id
    range_pub.radiation_type = 0
    range_pub.field_of_view = 0.34 #horizontal ~20, vertical ~13 -> radians 0.34
    range_pub.min_range = 0.02
    range_pub.max_range = 0.6
    range_pub.range = data


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard {}-{}-{}".format(data.data[0],data.data[1],data.data[2]))

    publishRange(pub_range1,data.data[0],"/sonyc_1")
    publishRange(pub_range1,data.data[1],"/sonyc_Mirror__1")
    publishRange(pub_range1,data.data[2],"/sonyc__1__1")
    
    pub.publish(r)
    rospy.sleep(1.0)

def main():
    rospy.init_node('convert_range_sensors_values', anonymous=True)
    rospy.loginfo("convert_range_sensors_values initialization ok!")
    
    rospy.Subscriber("rangeSonar1", Float32MultiArray, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass