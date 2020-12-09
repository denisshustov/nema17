#!/bin/bash

ex=$(rosversion -d)
 
if [[ $ex == 'melodic' ]]; 
 then 
	echo 'ROS melodic already installed!'
else 
	echo "Install ROS melodic"
	
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

	sudo apt update
	
	sudo apt install ros-melodic-desktop-full
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	
	sudo apt install python-rosdep
	sudo rosdep init
	rosdep update
	
	cd ~/
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	source devel/setup.bash
	cd ~/
fi

echo 'Installing packages'
sudo apt-get install python-catkin-tools

sudo apt-get install ros-melodic-rqt
sudo apt-get install ros-melodic-rqt-multiplot
sudo apt-get install ros-melodic-imu-tools

sudo apt-get install ros-melodic-map-server
sudo apt install ros-melodic-amcl
sudo apt-get install ros-melodic-robot-localization
sudo apt-get install ros-melodic-geographic-msgs
sudo apt-get install ros-melodic-move-base
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-pointcloud-to-laserscan
sudo apt-get install ros-melodic-global-planner
sudo apt-get install ros-melodic-navigation

sudo apt install ros-melodic-joint-state-publisher-gui
sudo apt-get install ros-melodic-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-velocity-controllers ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-teleop-twist-keyboard


sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial


# cd <sketchbook>/libraries
# rm -rf ros_lib
# rosrun rosserial_arduino make_libraries.py .

my_ip=$(ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
echo $my_ip

export ROS_IP=$my_ip
export ROS_MASTER_URI=http://localhost:11311

sudo apt install git
cd catkin_ws/src
git clone https://github.com/denisshustov/nema17
cd ..
catkin_make



























