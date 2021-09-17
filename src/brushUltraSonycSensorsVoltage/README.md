# brushUltraSonycSensorsVoltage

3_2.ino - arduino scatch for ultrasonic sensors and voltage measure to receive <br>
and subscribe funAndBrushes

rangeSonar is raw data for get normal data use convert_range_sensors_values [src/convert_range_sensors_values/README.md]

# Topics
public:
rangeSonar1 - Float32MultiArray, it contains 3 values<br>
voltage - Float32

subscribe:
funAndBrushes - ByteMultiArray, it receive 3 values, for two brushes and one turbine

# RUN
rosrun rosserial_python serial_node.py /dev/ttyUSB0<br>

run funAndBrushes:<br>
rostopic pub /funAndBrushes std_msgs/ByteMultiArray "layout:
  dim:
  - label: ''
    size: 3
    stride: 0
  data_offset: 0
data:
- 1
- 1
- 1" 

stop funAndBrushes:<br>
rostopic pub /funAndBrushes std_msgs/ByteMultiArray "layout:
  dim:
  - label: ''
    size: 3
    stride: 0
  data_offset: 0
data:
- 0
- 0
- 0" 


see topic
rostopic echo /rangeSonar1<br>
rostopic echo /voltage<br>



# Circumstance
<b>if add Kalman filter it works slow</b><br>

<b>if send in different topics it works slow</b><br>


ultrasonic sensors problems<br>
if error like:<br>
~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch <br>
... logging to /home/turtlebot3/.ros/log/50d0066c-1356-11e8-861b-a0c589238c19/roslaunch-turtlebot3-2515.log<br>
Checking log directory for disk usage. This may take awhile.<br>
Press Ctrl-C to interrupt<br>
Done checking log file disk usage. Usage is <1GB.<br>
<b>Solution:</b><br>
change arduino with more RAM or cut code for arduino<br>

# Links
https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/<br>
http://cobecoballes-robotics.blogspot.com/2018/08/sonar-ranger-ros-rviz.html<br>
http://wiki.ros.org/rosserial_arduino/Tutorials/SRF08%20Ultrasonic%20Range%20Finder<br>
