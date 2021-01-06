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
    wheel_radius = 0.074 / 2 #0.037
    wheelSep = 0.24
    RPM_TO_RAD_PER_S = (2 * PI)/60             # RPM per second
    DIST_PER_RAD = 2 * PI * wheel_radius
    R1 = ((2 * PI)/60) * wheel_radius           #0.003874631

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


    def run(self, rpm_left, rpm_right):
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
        
        #https://www.se.com/no/en/faqs/FA337686/
        # fz = RPM / ((a/360)*60)
        #*60!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! sek->min
        fz = (rpm_left*60) / 0.3 #rpm_left / 0.3

        self.pi.set_PWM_dutycycle(self.STEP1, 255)  # PWM 1/2 On 1/2 Off
        self.pi.set_PWM_frequency(self.STEP1, abs(fz))  # 500 pulses per second
        
        self.pi.set_PWM_dutycycle(self.STEP2, 255)  # PWM 1/2 On 1/2 Off
        self.pi.set_PWM_frequency(self.STEP2, abs(fz))  # 500 pulses per second
        
        self.pi.write(self.DIR1, left_w if self.CW else self.CCW)  # Set direction
        self.pi.write(self.DIR2, right_w if self.CW else self.CCW)  # Set direction
        
        real_fz = self.pi.get_PWM_frequency(self.STEP1)
        real_rpm =(real_fz/60) * 0.3        

        real_x = real_rpm * self.DIST_PER_RAD # ((self.DIST_PER_RAD * self.RPM_TO_RAD_PER_S ) * real_rpm)#* self.wheel_radius

        print("-------------------------------")
        print("real_x = {}".format(real_x))
        print("SET FZ = {}".format(fz))
        print("GET FZ = {}".format(real_fz))
        print("REAL RPM = {}".format(real_rpm))
       
        if left_w != right_w:
            move_cmd.angular.z = 60 * (real_x/self.DIST_PER_RAD) #real_x/((self.wheelSep / 2.0)) 
            if fz > 0:
                move_cmd.angular.z = move_cmd.angular.z * -1
            print("Z0 REAL SPEED = {}".format(move_cmd.angular.z))
        else:
            move_cmd.angular.z = 0
            if left_w and right_w:
                move_cmd.linear.x = real_x  * -1
            else:
                move_cmd.linear.x = real_x
        print("-------------------------------")
        
        self.real_cmd_vel_pub.publish(move_cmd)

class Driver:
    PI = 3.14159265359
    wheelSep = 0.24
    wheel_radius = 0.074 / 2 #0.037
    RPM_TO_RAD_PER_S = (2 * PI)/60             # RPM per second 0.104719755
    DIST_PER_RAD = 2 * PI * wheel_radius       #0.232477856
    R1 = ((2 * PI)/60) * wheel_radius

    def callback(self, msg):
        rospy.loginfo("Received a /ppp/cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (
            msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (
            msg.angular.x, msg.angular.y, msg.angular.z))
        
        #msg.angular.z radian/sek

        l_vel = msg.linear.x - ((self.wheelSep / 2.0) * msg.angular.z)# m/s
        r_vel = msg.linear.x + ((self.wheelSep / 2.0) * msg.angular.z)# m/s


        self._left_rpm = l_vel / self.DIST_PER_RAD#l_vel / (self.DIST_PER_RAD * self.RPM_TO_RAD_PER_S )# / self.wheel_radius
        self._right_rpm = r_vel / self.DIST_PER_RAD #r_vel / (self.DIST_PER_RAD * self.RPM_TO_RAD_PER_S )# / self.wheel_radius
        #((2xpi))x0.037

        # self._left_rpm = l_vel / self.R1
        # self._right_rpm = r_vel / self.R1


        print("!!! L SPEED = {}".format(l_vel))
        print("!!! R SPEED = {}".format(r_vel))
        print("self._left_rpm  = {}".format(self._left_rpm ))


    def __init__(self):
        rospy.init_node('driver')

        self.motor_driver = Motor_Driver(26, 19, 6,    16, 20, 21)
        
        self._left_rpm = 0
        self._right_rpm = 0


        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 1000)

        self.ros_sub_twist = rospy.Subscriber("/ppp/cmd_vel", Twist, self.callback)
        rospy.loginfo("Initialization complete")

    def run(self):

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            self.motor_driver.run(self._left_rpm, self._right_rpm)
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