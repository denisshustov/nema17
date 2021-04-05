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
#sudo apt-get install python-catkin-tools ros-melodic-rqt ros-melodic-rqt-multiplot ros-melodic-imu-tools ros-melodic-map-server ros-melodic-amcl ros-melodic-robot-localization ros-melodic-geographic-msgs ros-melodic-move-base ros-melodic-map-server ros-melodic-pointcloud-to-laserscan ros-melodic-global-planner ros-melodic-navigation ros-melodic-joint-state-publisher-gui ros-melodic-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-velocity-controllers ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control libqt5webkit5 libqt5webkit5-dev qt5-default ros-melodic-teleop-twist-keyboard ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-rosserial-arduino ros-melodic-rosserial -y
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-multiplot ros-noetic-imu-tools ros-noetic-map-server ros-noetic-amcl ros-noetic-robot-localization ros-noetic-geographic-msgs ros-noetic-move-base ros-noetic-map-server ros-noetic-pointcloud-to-laserscan ros-noetic-global-planner ros-noetic-navigation  ros-noetic-joint-state-publisher-gui ros-noetic-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers ros-noetic-velocity-controllers ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control libqt5webkit5 libqt5webkit5-dev ros-noetic-teleop-twist-keyboard ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-rosserial-arduino ros-noetic-rosserial -y
sudo apt install qt5-default qtscript5-dev libqwt-qt5-dev
sudo add-apt-repository ppa:rock-core/qt4
sudo apt update
sudo apt install qt4-default
sudo apt install python3-pip
pip3 install scikit-image

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



























