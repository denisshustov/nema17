#!/usr/bin/env python


import time
import sys
from time import sleep
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist
import math
import pigpio

class Motor_Driver:
    CW = 1
    CCW = 0

    SPR = 200   # Steps per Revolution (360 / 1.8)    
    PI = 3.14159265359

    def __init__(self, dir1, step1, en1, dir2, step2, en2):
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
        self.pi = pigpio.pi()
        self.real_cmd_vel_pub = rospy.Publisher("/ppp/real_cmd_vel", Twist, queue_size = 500)        


    def run(self, rpm_left, rpm_right, z):
        move_cmd = Twist()

        if rpm_left == 0 and rpm_right == 0:
            if(self.prev_rpm_left != rpm_left and self.prev_rpm_right != rpm_right):
                rospy.loginfo("Disabled drivers, speed 0!")
                GPIO.output(self.EN1, GPIO.HIGH)
                GPIO.output(self.EN2, GPIO.HIGH)
                self.prev_rpm_left = rpm_left
                self.prev_rpm_right = rpm_right
                move_cmd.angular.z = 0
                move_cmd.linear.x = 0
                self.real_cmd_vel_pub.publish(move_cmd)
            return
        else:
            if(self.prev_rpm_left != rpm_left and self.prev_rpm_right != rpm_right):
                rospy.loginfo("Enabled drivers, rpm "+str(rpm_left))
                GPIO.output(self.EN1, GPIO.LOW)
                GPIO.output(self.EN2, GPIO.LOW)

        self.prev_rpm_left = rpm_left
        self.prev_rpm_right = rpm_right
        
        start = time.time()
        left_w = rpm_left < 0
        right_w = rpm_right < 0

#5:  8000  4000  2000 1600 1000  800  500  400  320
#   250   200   160  100   80   50   40   20   10        
# sudo systemctl stop pigpiod.service
# sudo pigpiod

# RPM = (step angle)/360 * fz * 60 =>0,005 * fz * 60=>fz * 0.3
# fz = RPM / 0,005 * 60 => RPM / 0,3

# V = ((2*PI)/60) * Radius * RPM; Raduis=0.037;((2*PI)/60)=0.104719755
# V = 0.104719755 * 0.037 * RPM=> 0.003874631 * RPM

#0.3 * 10 = 3 * 0.003874631 =>0.011623893 m/s
#0.3 * 20 = 6 * 0.003874631=>0.023247786 m/s
#0.3 * 40 = 12 * 0.003874631=>0.046495572 m/s
#0.3 * 50 = 15 * 0.003874631=>0.058119465 m/s
#0.3 * 80 = 24 * 0.003874631=>0.092991144 m/s
#0.3 * 100 = 30* 0.003874631 =>0.11623893 m/s
#0.3 * 160 = 48 * 0.003874631 =>0.185982288 m/s
#0.3 * 200 = 60 * 0.003874631 => 0.23247786 m/s
#0.3 * 250 = 75 * 0.003874631 => 0.290597325 m/s
#0.3 * 320 = 96 * 0.003874631 => 0.371964576 m/s
#0.3 * 400 = 120 * 0.003874631 => 0.46495572 m/s

        fz = rpm_left / 0.3

        self.pi.set_PWM_dutycycle(self.STEP1, 128)  # PWM 1/2 On 1/2 Off
        self.pi.set_PWM_frequency(self.STEP1, abs(fz))  # 500 pulses per second
        
        self.pi.set_PWM_dutycycle(self.STEP2, 128)  # PWM 1/2 On 1/2 Off
        self.pi.set_PWM_frequency(self.STEP2, abs(fz))  # 500 pulses per second
        
        self.pi.write(self.DIR1, left_w if self.CW else self.CCW)  # Set direction
        self.pi.write(self.DIR2, right_w if self.CW else self.CCW)  # Set direction
        

        wheel_radius = 0.074 / 2 #0.037
        wheelSep = 0.24
        rospy.loginfo("SET FZ = {}".format(fz))
        real_fz = self.pi.get_PWM_frequency(self.STEP1)
        real_rpm = real_fz * 0.005
        real_x = (2 * 3.14159265359) * wheel_radius * real_rpm

        print("GET FZ = {}".format(real_fz))
        print("X REAL SPEED = {}".format(real_x))

#velDiff = ( 0.24 * msg.angular.z) / 2.0
#self._left_rpm = (msg.linear.x - velDiff) / (0.037 * ((2 * self.PI) / 60))
#self._right_rpm = (msg.linear.x + velDiff) / (0.037 * ((2 * self.PI) / 60))
#((2*PI)/60)=0.104719755 
#0.104719755 * 0.037 = 0.003874631

#!msg.linear.x = 0
#self._left_rpm = ( -1 * ( 0.24 * msg.angular.z) / 2.0) / 0.003874631
#self._right_rpm = (( 0.24 * msg.angular.z) / 2.0 ) / 0.003874631
            
        
#0.037 * (2pi/60)=0.003874631
        prc = (real_fz * 100)/ fz
        

        RP = ((2 * 3.14159265359)/60) * wheel_radius
        if left_w != right_w:
            #(real_rpm * RP * 2) / wheelSep#(0.037 *((2 * 3.14159265359) / 60)*2) / real_rpm * 0.24  #(( 0.24 * real_x) / 2.0 ) / 0.003874631
            move_cmd.angular.z = z #* (prc/100)
            print("prc = {}".format(prc))
            if not right_w:
                move_cmd.angular.z = move_cmd.angular.z * -1
            print("Z REAL SPEED = {}".format(move_cmd.angular.z))
        else:
            move_cmd.angular.z = 0
            if left_w and right_w:
                move_cmd.linear.x = real_x  * -1
            else:
                move_cmd.linear.x = real_x
        
        # rospy.loginfo("q = {}".format(move_cmd.angular.z))
        # rospy.loginfo("left rpm = {}".format(self._left_rpm))

        self.real_cmd_vel_pub.publish(move_cmd)

        #end = time.time()


class Driver:
    PI = 3.14159265359

    def callback(self, msg):
        rospy.loginfo("Received a /ppp/cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (
            msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (
            msg.angular.x, msg.angular.y, msg.angular.z))
        
        velDiff = ((self.wheelSep)/ 2.0) * msg.angular.z
# vel_l = ((msg.linear.x - (msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
# vel_r = ((msg.linear.x + (msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

        # self._left_speed = ((msg.linear.x - (msg.angular.z * self.wheelSep / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
        # self._right_speed =  ((msg.linear.x + (msg.angular.z * self.wheelSep / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

        #self._left_rpm = (msg.linear.x + velDiff) / self.wheel_radius
        #self._right_rpm = (msg.linear.x - velDiff) / self.wheel_radius

        self.z = msg.angular.z
        #self._left_rpm = (msg.linear.x - velDiff) / (self.wheel_radius * (60 / (2 * self.PI) ))
        #self._right_rpm = (msg.linear.x + velDiff) / (self.wheel_radius * (60 / (2 * self.PI)))
        print("WHANT SPEED = {}".format(msg.linear.x))
        #self._left_rpm = (msg.linear.x) / (self.wheel_radius * ((2 * self.PI) / 60))
        #self._right_rpm = (msg.linear.x) / (self.wheel_radius * ((2 * self.PI) / 60))
        ???????????????????????????????????????/
        ???????????????????????????????????????/
        ???????????????????????????????????????/
        self._left_rpm = (msg.linear.x - msg.angular.z) *20
        self._right_rpm = (msg.linear.x + msg.angular.z)*20

    def __init__(self):
        rospy.init_node('driver')

        self.motor_driver = Motor_Driver(
            26, 19, 6,    16, 20, 21)

        self.z = 0
        self._left_rpm = 0
        self._right_rpm = 0
        self.wheelSep = 0.24
        self.wheel_radius = 0.074 / 2 #0.037

        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 1000)

        self.ros_sub_twist = rospy.Subscriber("/ppp/cmd_vel", Twist, self.callback)
        rospy.loginfo("Initialization complete")

    def run(self):

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            self.motor_driver.run(self._left_rpm, self._right_rpm, self.z)
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
