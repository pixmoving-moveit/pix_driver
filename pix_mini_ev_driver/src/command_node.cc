#include "ros/ros.h"
#include "can_msgs/Frame.h"


// include- msgs header file
// Example: #include "pix_mini_ev_driver_msgs/BrakeCommand.h"
// #include pix_mini_ev_driver_msgs/protocols["name"].h
#include "pix_mini_ev_driver_msgs/msg100_100.h"
#include "pix_mini_ev_driver_msgs/msg101_101.h"
#include "pix_mini_ev_driver_msgs/msg104_104.h"



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include "msg100_100.hpp"
#include "msg101_101.hpp"
#include "msg104_104.hpp"



// global variable: msg
/*
Example:
static can_msgs::Frame can_brake;
static pix_mini_ev_driver_msgs::BrakeCommand brake_command_msg;

static can_msgs::Frame can_protocols["name"].split("_",1)[0];
static pix_mini_ev_driver_msgs::protocols["name"] protocols["name"]_msg
*/
static pix_mini_ev_driver_msgs::msg100_100 msg100_100_msg;
static pix_mini_ev_driver_msgs::msg101_101 msg101_101_msg;
static pix_mini_ev_driver_msgs::msg104_104 msg104_104_msg;

static can_msgs::Frame can_msg100;
static can_msgs::Frame can_msg101;
static can_msgs::Frame can_msg104;


// global variable:  send entity
// Example: static Brakecommand101 brake_command;
// static protocols["name"] protocols["name"]_entity;
static Msg100100 msg100_100_entity;
static Msg101101 msg101_101_entity;
static Msg104104 msg104_104_entity;



// global variable: Enable and send time stamp
/*
Example:
static bool brake_enable, gear_enable, park_enable, steer_enable, throttle_enable;
static int brake_prev_t=0, gear_prev_t=0, park_prev_t=0, steer_prev_t=0, throttle_prev_t=0;

protocols["name"].split("_",1)[0]_enable, ...
protocols["name"].split("_",1)[0]_prev_t=0, ...
*/
static bool msg100_enable, msg101_enable, msg104_enable;
static int msg100_prev_t=0, msg101_prev_t=0, msg104_prev_t=0;


int time_diff = 200000;
static ros::Publisher pub_can;


// define send canID callback function

static void msg100_callback(const pix_mini_ev_driver_msgs::msg100_100 &msg)
{
    
    msg100_100_entity.Reset();
    msg100_100_msg = msg;
    can_msg100.header.stamp = ros::Time::now();
    can_msg100.dlc = 8;
    msg100_100_entity.UpdateData(
    msg100_100_msg.AutoCtrlEna,
	msg100_100_msg.ModeCtrlCnt,
	msg100_100_msg.ModeCtrlCks
    );
    can_msg100.id = msg100_100_entity.ID;
    can_msg100.is_extended= false;
    uint8_t *A;
    A = msg100_100_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_msg100.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_msg100.header.stamp.toNSec();
    msg100_prev_t = t_nsec;
}

    
static void msg101_callback(const pix_mini_ev_driver_msgs::msg101_101 &msg)
{
    
    msg101_101_entity.Reset();
    msg101_101_msg = msg;
    can_msg101.header.stamp = ros::Time::now();
    can_msg101.dlc = 8;
    msg101_101_entity.UpdateData(
    msg101_101_msg.AccPedCmd,
	msg101_101_msg.AccCtrlEna,
	msg101_101_msg.AccPedInv,
	msg101_101_msg.AccCtrlCnt,
	msg101_101_msg.AccCtrlCks,
	msg101_101_msg.AccTkoDis
    );
    can_msg101.id = msg101_101_entity.ID;
    can_msg101.is_extended= false;
    uint8_t *A;
    A = msg101_101_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_msg101.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_msg101.header.stamp.toNSec();
    msg101_prev_t = t_nsec;
}

    
static void msg104_callback(const pix_mini_ev_driver_msgs::msg104_104 &msg)
{
    
    msg104_104_entity.Reset();
    msg104_104_msg = msg;
    can_msg104.header.stamp = ros::Time::now();
    can_msg104.dlc = 8;
    msg104_104_entity.UpdateData(
    msg104_104_msg.WiperCmd,
	msg104_104_msg.HornCmd,
	msg104_104_msg.GearCmd,
	msg104_104_msg.GearCtrlEna,
	msg104_104_msg.TurnLightCmd,
	msg104_104_msg.BeamLightCmd,
	msg104_104_msg.GearCtrlCnt,
	msg104_104_msg.GearCtrlCks
    );
    can_msg104.id = msg104_104_entity.ID;
    can_msg104.is_extended= false;
    uint8_t *A;
    A = msg104_104_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_msg104.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_msg104.header.stamp.toNSec();
    msg104_prev_t = t_nsec;
}

    


//  define timer callback function
void timer_callback(const ros::TimerEvent &te)
{
    int time_diff;
    int now;
    now = ros::Time::now().toNSec();
    /*Example: 
    // brake
    if(now-brake_prev_t>30000000)
    {
        for(uint i=0;i<8;i++)
        {   
            can_brake.id = brake_command.ID;
            can_brake.data[i] = 0;
        }
        pub_can.publish(can_brake);
    }
    else{
        pub_can.publish(can_brake);   
    }
    */
    
    // msg100
    if(now - msg100_prev_t>30000000)
    {
        for(uint i=0; i<8; i++)
        {   
            can_msg100.id = msg100_100_entity.ID;
            can_msg100.data[i] = 0;
        }
        pub_can.publish(can_msg100);
    }
    else{
        pub_can.publish(can_msg100);
    }
    
    // msg101
    if(now - msg101_prev_t>30000000)
    {
        for(uint i=0; i<8; i++)
        {   
            can_msg101.id = msg101_101_entity.ID;
            can_msg101.data[i] = 0;
        }
        pub_can.publish(can_msg101);
    }
    else{
        pub_can.publish(can_msg101);
    }
    
    // msg104
    if(now - msg104_prev_t>30000000)
    {
        for(uint i=0; i<8; i++)
        {   
            can_msg104.id = msg104_104_entity.ID;
            can_msg104.data[i] = 0;
        }
        pub_can.publish(can_msg104);
    }
    else{
        pub_can.publish(can_msg104);
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_mini_ev_driver_command_node");
    ros::NodeHandle nh;

    // creat ros Subscriber
    // Example: ros::Subscriber sub_brake = nh.subscribe("/pix/brake_command", 1, brake_callback);
    ros::Subscriber sub_msg100 = nh.subscribe("/pix/msg100", 1, msg100_callback);
	ros::Subscriber sub_msg101 = nh.subscribe("/pix/msg101", 1, msg101_callback);
	ros::Subscriber sub_msg104 = nh.subscribe("/pix/msg104", 1, msg104_callback);
	
    //  creat ros publisher
    pub_can = nh.advertise<can_msgs::Frame>("/sent_messages", 5, true);

    ros::Timer set_speed = nh.createTimer(ros::Duration(1/50.0), timer_callback);
    ros::spin();

    return 0;

}