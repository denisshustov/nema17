#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
from time import sleep
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int32, Float64


class Motor_Driver:
    CW = 1     # Clockwise Rotation
    CCW = 0    # Counterclockwise Rotation

    SPR = 200   # Steps per Revolution (360 / 1.8)
    delay_const = .00243

    def __init__(self, dir, step, en):
        self.delay = self.delay_const
        self.DIR = dir
        self.STEP = step
        self.EN = en
        self.prev_rpm = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR, GPIO.OUT)
        GPIO.setup(self.STEP, GPIO.OUT)

        GPIO.setup(self.EN, GPIO.OUT)
        GPIO.output(self.EN, GPIO.HIGH)
        rospy.loginfo("Disabled drivers, speed 0!")

        GPIO.setmode(GPIO.BCM)

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

    def run(self, rpm):

        if rpm == 0:
            if self.prev_rpm != rpm:
                rospy.loginfo("Disabled drivers, speed 0!")
                GPIO.output(self.EN, GPIO.HIGH)
                self.prev_rpm = rpm
            return
        else:
            if self.prev_rpm != rpm:
                rospy.loginfo("Enabled drivers, rpm "+str(rpm))
                GPIO.output(self.EN, GPIO.LOW)

        self.prev_rpm = rpm

        start = time.time()

        GPIO.output(self.DIR, rpm > 0 if self.CW else self.CCW)

        rpm = abs(rpm)
        self.delay = self.__rpm_to_delay(rpm)

        GPIO.output(self.STEP, GPIO.HIGH)

        sleep(self.delay)

        GPIO.output(self.STEP, GPIO.LOW)

        sleep(self.delay)

        end = time.time()
        # print(end-start)


class Driver:
   # '''
   # v = r x RPM x 0.10472 => v/RPM = r  x 0.10472 => RPM = (r x 0.10472) / v
   # v: Linear velocity, in m/s
   # r: Radius, in meter
   # RPM: Angular velocity, in RPM (Rounds per Minute)
   # '''

    def right_callback(self, msg):
        if msg.data==0:
            self.right_motor_driver.run(0)
        else:
            rpm = (self._wheel_radius * 0.10472) / msg.data
            rospy.loginfo("Right data [%f], RPM [%f]" % (msg.data, rpm))
            self.right_motor_driver.run(rpm)

    def left_callback(self, msg):
        if msg.data==0:
            self.right_motor_driver.run(0)
        else:
            rpm = (self._wheel_radius * 0.10472) / msg.data
            rospy.loginfo("Right data [%f], RPM [%f]" % (msg.data, rpm))
            self.left_motor_driver.run(rpm)

    def __init__(self):
        rospy.init_node('driver')

        self.right_motor_driver = Motor_Driver(26, 19, 6)

        self.left_motor_driver = Motor_Driver(16, 20, 21)

        self._last_received = rospy.get_time()
        self._wheel_radius = rospy.get_param('~wheel_radius_meters', 2)
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 1000)

        self.ros_sub_right = rospy.Subscriber("/ppp/right_motor_control", Float64, self.right_callback) #m/s
        self.ros_sub_left = rospy.Subscriber("/ppp/left_motor_control", Float64, self.left_callback)
        rospy.loginfo("Initialization complete")
        

    def run(self):
        rospy.spin()


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
