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
from threading import Thread

class Motor_Driver():
    CW = 1     # Clockwise Rotation
    CCW = 0    # Counterclockwise Rotation

    SPR = 200   # Steps per Revolution (360 / 1.8)
    delay_const = .00243

    def __init__(self, dir, step, en, name, topic_name, wheel_radius, rate, with_debug_info):
        self.delay = self.delay_const
        self.DIR = dir
        self.STEP = step
        self.EN = en
        self.prev_rpm = 0
        self.NAME  = name
        self.TOPIC_NAME = topic_name
        self.rpm = 0
        self.WITH_DEBUG_INFO = with_debug_info

        GPIO.setup(self.DIR, GPIO.OUT)
        GPIO.setup(self.STEP, GPIO.OUT)

        GPIO.setup(self.EN, GPIO.OUT)
        GPIO.output(self.EN, GPIO.HIGH)
        if self.WITH_DEBUG_INFO:
            rospy.loginfo("Disabled motor, speed 0! {}".format(self.TOPIC_NAME))

        self._last_received = rospy.get_time()
        self._wheel_radius = wheel_radius
        self._rate = rate


    def run(self):
        self.ros_sub_right = rospy.Subscriber(self.TOPIC_NAME , Float64, self.callback) #m/s
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            delay = rospy.get_time() - self._last_received
            self.run_motor(self.rpm)
            rate.sleep()

    def callback(self, msg):
        if msg.data==0:
            self.rpm = 0
        else:
            self.rpm = msg.data / (self._wheel_radius * 0.10472)
            if self.WITH_DEBUG_INFO:
                rospy.loginfo(self.NAME + " data [%f], RPM [%f]" % (msg.data, self.rpm))

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

    def run_motor(self, rpm):

        if rpm == 0:
            if self.prev_rpm != rpm:
                if self.WITH_DEBUG_INFO:
                    rospy.loginfo("Disabled motor, speed 0! {}".format(self.TOPIC_NAME))
                GPIO.output(self.EN, GPIO.HIGH)
                self.prev_rpm = rpm
            return
        else:
            if self.prev_rpm != rpm:
                if self.WITH_DEBUG_INFO:
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
   # v = r x RPM x 0.10472 => RPM = v/(r x 0.10472) 
   # v: Linear velocity, in m/s
   # r: Radius, in meter
   # RPM: Angular velocity, in RPM (Rounds per Minute)
   # '''

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        rospy.init_node('motor_control')

        dir = rospy.get_param('~dir')
        step = rospy.get_param('~step')
        en = rospy.get_param('~en')
        name = rospy.get_param('~name')
        topic_name = rospy.get_param('~topic_name')
        wheel_radius = rospy.get_param('~wheel_radius_meters', 0.073)
        rate = rospy.get_param('~rate', 1000)
        with_debug_info = rospy.get_param('~with_debug_info', True)

        if dir == None or step == None or en == None or name == None or topic_name == None:
            raise Exception('Wrong node params!!!')
            return

        self.motor_driver = Motor_Driver(dir, step, en, name, topic_name, wheel_radius, rate, with_debug_info)
        self.motor_driver.run()

        rospy.loginfo("Initialization complete")
        


def main():
    try:
        driver = Driver()

    except AttributeError as error:
        print(error)
    except NameError as e:
        print e
        print sys.exc_type
    except:
        print("Unexpected error:", sys.exc_info()[0])
    finally:
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(21, GPIO.HIGH)
        GPIO.cleanup()


if __name__ == '__main__':
    main()
