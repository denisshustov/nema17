#!/usr/bin/env python


import time
import sys
from time import sleep
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist


class Motor_Driver:
    CW = 1     # Clockwise Rotation
    CCW = 0    # Counterclockwise Rotation

    SPR = 200   # Steps per Revolution (360 / 1.8)
    delay_const = .00243

    RESOLUTION = {'Full': (0, 0, 0),
                  'Half': (1, 0, 0),
                  '1/4': (0, 1, 0),
                  '1/8': (1, 1, 0),
                  '1/16': (0, 0, 1),
                  '1/32': (1, 0, 1)}

    def __init__(self, dir1, step1, mode1, dir2, step2, mode2):
        self.delay = self.delay_const
        self.DIR1 = dir1
        self.STEP1 = step1
        self.DIR2 = dir2
        self.STEP2 = step2

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR1, GPIO.OUT)
        GPIO.setup(self.STEP1, GPIO.OUT)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR2, GPIO.OUT)
        GPIO.setup(self.STEP2, GPIO.OUT)

        # (14, 15, 18)   # Microstep Resolution GPIO Pins
        self.MODE1 = mode1
        GPIO.setup(self.MODE1, GPIO.OUT)

        # (14, 15, 18)   # Microstep Resolution GPIO Pins
        self.MODE2 = mode2
        GPIO.setup(self.MODE2, GPIO.OUT)

        GPIO.output(self.MODE1, self.RESOLUTION['Full'])
        GPIO.output(self.MODE2, self.RESOLUTION['Full'])

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

    def run(self, rpm1, rpm2):  # first motor, second motor

        #print('rpm1'+str(rpm1))
        #print('rpm2'+str(rpm2))

        if rpm1 == 0 and rpm2 == 0:
            return

        start = time.time()

        GPIO.output(self.DIR1, rpm1 > 0 if self.CW else self.CCW)
        GPIO.output(self.DIR2, rpm2 > 0 if self.CW else self.CCW)

        rpm1 = abs(rpm1)
        rpm2 = abs(rpm2)

        self.delay = self.__rpm_to_delay(rpm1)

        #for x in range(self.SPR):
        GPIO.output(self.STEP1, GPIO.HIGH)
        GPIO.output(self.STEP2, GPIO.HIGH)
 
        sleep(self.delay)
 
        GPIO.output(self.STEP1, GPIO.LOW)
        GPIO.output(self.STEP2, GPIO.LOW)

        sleep(self.delay)

        end = time.time()
        print(end-start)


class Driver:
    def callback(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (
            msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (
            msg.angular.x, msg.angular.y, msg.angular.z))

        linear = msg.linear.x
        angular = msg.angular.z

        # Calculate wheel speeds in m/s
        self._left_speed = linear - angular
        self._right_speed = linear + angular

    def __init__(self):
        rospy.init_node('driver')

        self.motor_driver = Motor_Driver(
            20, 21, (14, 15, 18), 26, 19, (6, 5, 13))

        self._left_speed = 0
        self._right_speed = 0

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 1000)

        self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.loginfo("Initialization complete")
        #rospy.spin()

    def run(self):

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            #print('speed '+str(self._left_speed)+', '+str(self._right_speed))
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                self.motor_driver.run(self._left_speed, self._right_speed)
            else:
                self.motor_driver.run(self._left_speed, self._right_speed)
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
