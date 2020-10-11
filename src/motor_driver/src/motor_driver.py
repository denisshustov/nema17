#!/usr/bin/env python


import time
import sys
from time import sleep
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist
import math

class Motor_Driver:
    CW = 1     # Clockwise Rotation
    CCW = 0    # Counterclockwise Rotation

    SPR = 200   # Steps per Revolution (360 / 1.8)
    delay_const = .00243

    def __init__(self, dir1, step1, en1, dir2, step2, en2):
        self.delay = self.delay_const
        self.DIR1 = dir1
        self.STEP1 = step1
        self.DIR2 = dir2
        self.STEP2 = step2
        self.EN1 = en1
        self.EN2 = en2
        self.prev_rpm_left = 0
        self.prev_rpm_right = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR1, GPIO.OUT)
        GPIO.setup(self.STEP1, GPIO.OUT)

        GPIO.setup(self.EN1, GPIO.OUT)
        GPIO.setup(self.EN2, GPIO.OUT)

        GPIO.output(self.EN1, GPIO.HIGH)
        GPIO.output(self.EN2, GPIO.HIGH)
        rospy.loginfo("Disabled drivers, speed 0!")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR2, GPIO.OUT)
        GPIO.setup(self.STEP2, GPIO.OUT)

    def __rpm_to_delay(self, rpm):
        if rpm == 0:
            return 0
        delay = 0
        if rpm >= 60:
            if rpm < 240:  # 240 max
                delay = self.delay_const/(rpm/60)
            else:
                delay = self.delay_const/4
        else:
            delay = self.delay_const*(60/rpm)
        return delay

    def run(self, rpm_left, rpm_right):  # first motor, second motor

        # print('rpm_left'+str(rpm_left))
        # print('rpm_right'+str(rpm_right))

        if rpm_left == 0 and rpm_right == 0:
            if(self.prev_rpm_left != rpm_left and self.prev_rpm_right != rpm_right):
                rospy.loginfo("Disabled drivers, speed 0!")
                GPIO.output(self.EN1, GPIO.HIGH)
                GPIO.output(self.EN2, GPIO.HIGH)
                self.prev_rpm_left = rpm_left
                self.prev_rpm_right = rpm_right
            return
        else:
            if(self.prev_rpm_left != rpm_left and self.prev_rpm_right != rpm_right):
                rospy.loginfo("Enabled drivers, rpm "+str(rpm_left))
                GPIO.output(self.EN1, GPIO.LOW)
                GPIO.output(self.EN2, GPIO.LOW)

        self.prev_rpm_left = rpm_left
        self.prev_rpm_right = rpm_right

        start = time.time()

        GPIO.output(self.DIR1, rpm_left > 0 if self.CW else self.CCW)
        GPIO.output(self.DIR2, rpm_right > 0 if self.CW else self.CCW)

        rpm_left = abs(rpm_left)
        rpm_right = abs(rpm_right)

        self.delay = self.__rpm_to_delay(rpm_left)

        # for x in range(self.SPR):
        GPIO.output(self.STEP1, GPIO.HIGH)
        GPIO.output(self.STEP2, GPIO.HIGH)

        sleep(self.delay)

        GPIO.output(self.STEP1, GPIO.LOW)
        GPIO.output(self.STEP2, GPIO.LOW)

        sleep(self.delay)

        end = time.time()
        # print(end-start)


class Driver:
    def callback(self, msg):
        rospy.loginfo("Received a /ppp/cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (
            msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (
            msg.angular.x, msg.angular.y, msg.angular.z))

        linear = msg.linear.x
        angular = math.degrees(msg.angular.z)  #radians -> degrees

        # Calculate wheel speeds in m/s
        self._left_speed = ((angular*0.29)/2)+linear
        self._right_speed = ((linear*2) - self._left_speed)

    def __init__(self):
        rospy.init_node('driver')

        self.motor_driver = Motor_Driver(
            26, 19, 6,    16, 20, 21)

        self._left_speed = 0
        self._right_speed = 0

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 1000)

        self.ros_sub_twist = rospy.Subscriber("/ppp/cmd_vel", Twist, self.callback)
        rospy.loginfo("Initialization complete")
        # rospy.spin()

    def run(self):

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            #print('speed '+str(self._left_speed)+', '+str(self._right_speed))
            delay = rospy.get_time() - self._last_received

            self.motor_driver.run(self._left_speed*-1, self._right_speed*-1)
            rate.sleep()


def main():
    try:
        driver = Driver()

        driver.run()
    except AttributeError as error:
        print(error)
    except NameError as e:
        print e
        print sys.exc_type
    except:
        print("Unexpected error:", sys.exc_info()[0])
    finally:
        GPIO.output(16, GPIO.HIGH)
        GPIO.output(12, GPIO.HIGH)
        GPIO.cleanup()


if __name__ == '__main__':
    main()

# . ~/catkin_ws/devel/setup.bash
# source devel/setup.sh
# rospack depends motor_driver
# rosrun motor_driver motor_driver_node.py
# 192.168.99.253
# export ROS_MASTER_URI=http://192.168.99.253:11311
# export ROS_IP=192.168.99.253
