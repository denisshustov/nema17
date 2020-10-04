# nema17

<b>#3_2.ino - arduino scatch for ultrasonic sensors</b><br>
<br>
links:
https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/<br>
http://cobecoballes-robotics.blogspot.com/2018/08/sonar-ranger-ros-rviz.html<br>
http://wiki.ros.org/rosserial_arduino/Tutorials/SRF08%20Ultrasonic%20Range%20Finder<br>

if add Kalman filter it works slow<br>
if send in different topics it works slow<br>

<b>RUN</b><br>
rosrun rosserial_python serial_node.py /dev/ttyUSB0<br>
rostopic echo /rangeSonar1<br>

#ultrasonic sensors problems<br>
if error like:<br>
~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch <br>
... logging to /home/turtlebot3/.ros/log/50d0066c-1356-11e8-861b-a0c589238c19/roslaunch-turtlebot3-2515.log<br>
Checking log directory for disk usage. This may take awhile.<br>
Press Ctrl-C to interrupt<br>
Done checking log file disk usage. Usage is <1GB.<br>
<br>
<br>
Solution:<br>
change arduino with more RAM or cut code for arduino<br>
