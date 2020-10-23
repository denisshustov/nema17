#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Twist.h>

class pppHW: public hardware_interface::RobotHW
{

//1.8 degree per tick
#define TICKS_PER_REVOLUTION 200
#define REVOLUTIONS_PER_TICK 0.005
public:
    pppHW(ros::NodeHandle nh)
    {
        //ros::NodeHandle pnh("~");???
        ROS_INFO("Init HW controller");

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_left("Rev1", &pos[0], &vel[0],&eff[0]);
        jnt_state_interface.registerHandle(state_handle_left);

        hardware_interface::JointStateHandle state_handle_right("Rev4", &pos[1], &vel[1],&eff[1]);
        jnt_state_interface.registerHandle(state_handle_right);

        registerInterface(&jnt_state_interface);


        // connect and register the joint position interface
        hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("Rev1"), &cmd[0]);
        jnt_vel_interface.registerHandle(vel_handle_left);

        hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("Rev4"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_right);

        registerInterface(&jnt_vel_interface);


        motor_left_pub = nh.advertise<std_msgs::Float64>("/ppp/right_motor_control",10);
        motor_right_pub = nh.advertise<std_msgs::Float64>("/ppp/left_motor_control",10);
        //ppp_odom_sub = nh.subscribe<nav_msgs::Odometry>("/ppp/odom",10, &pppHW::pppOdomCallBack, this);

        prev_left=0;
        prev_right=0;
        pos[0]=0;
        pos[1]=0;

        ROS_INFO("!!!HW LOAD OK!!!");
    }

    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

    // void pppOdomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
    // {
    //     odomData = msg->data;
    // }
 
    void read()
    {
        left_msg_data.data=cmd[0];
        right_msg_data.data=cmd[1];
        motor_left_pub.publish(left_msg_data);
        motor_right_pub.publish(right_msg_data);
    }

    void write()
    {
        vel[0] = REVOLUTIONS_PER_TICK * (cmd[0] - prev_left) / getPeriod().toSec();
        vel[1] = REVOLUTIONS_PER_TICK * (cmd[1] - prev_right) / getPeriod().toSec();
        pos[0] = REVOLUTIONS_PER_TICK * (cmd[0] - prev_left);
        pos[1] = REVOLUTIONS_PER_TICK * (cmd[1] - prev_right);
        prev_left = cmd[0];
        prev_right = cmd[1];
        ROS_INFO(">>>WH vel  = %.2f, %.2f", vel[0], vel[1]);
    }

    private:
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::JointStateInterface jnt_state_interface;

    //controllers will read from the pos, vel and eff variables in your robot
    //make sure the pos, vel and eff variables always have the latest joint state available
    double pos[2];
    double vel[2];
    double eff[2];

    //controller will write the desired command into the cmd variable
    //make sure that whatever is written into the cmd variable gets executed by the robot
    double cmd[2];

    std_msgs::Float64 left_msg_data, right_msg_data;
    double prev_left, prev_right;
    ros::Publisher motor_left_pub, motor_right_pub;
    ros::Subscriber ppp_odom_sub;
    //nav_msgs::Odometry odomData;

    
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "hw_control");
    ros::NodeHandle nh;
    pppHW pppHw(nh);
    controller_manager::ControllerManager cm(&pppHw, nh);

    ros::Rate rate(1.0/pppHw.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok)
    {
        pppHw.read();
        cm.update(pppHw.getTime(), pppHw.getPeriod());
        pppHw.write();
        rate.sleep();            
    }
    spinner.stop();
    return 0;
}