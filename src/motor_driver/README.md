# motor_driver

# Code 
motor_driver.py
motor-driver.launch

and launch in reapberry

# How it works
Subscribe topic /ppp/cmd_vel and translate it to velocity for eatch motor<br>
and publish topic /ppp/real_cmd_vel, b/c it can't make actually values that sended by cmd_vel<br>
It convert linear speed to rpm:

```python
l_vel = msg.linear.x - ((self.wheelSep / 2.0) * msg.angular.z)# m/s
r_vel = msg.linear.x + ((self.wheelSep / 2.0) * msg.angular.z)# m/s

self._left_rpm = l_vel / self.DIST_PER_RAD
self._right_rpm = r_vel / self.DIST_PER_RAD
```
wheelSep - distance between wheels<br>
DIST_PER_RAD - it distance per radian<br>

And in Motor_Driver.run receive rpm_left, rpm_right<br>
witch convert to frequency<br>

```python
fz_left = (rpm_left * 60) / 0.3
fz_right = (rpm_right * 60) / 0.3
```
this magic number get from https://www.se.com/no/en/faqs/FA337686/<br>
and run it.<br>
But it runs not with correct frequency. Correct frequency get by<br>

```python
real_fz_left = self.pi.get_PWM_frequency(self.STEP1)
real_fz_right = self.pi.get_PWM_frequency(self.STEP2)

real_fz_left = real_fz_left * math.copysign(1, fz_left)
real_fz_right = real_fz_right * math.copysign(1, fz_right)

real_rpm_left = (real_fz_left * 0.3) / 60
real_rpm_right = (real_fz_right * 0.3) / 60

real_velocity_left = real_rpm_left * self.DIST_PER_RAD
real_velocity_right = real_rpm_right * self.DIST_PER_RAD
```
and convert it to velocity


# Links
https://www.rototron.info/raspberry-pi-stepper-motor-tutorial/
https://www.se.com/no/en/faqs/FA337686/