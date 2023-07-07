#include "ros/ros.h"
#include "can_msgs/Frame.h"


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include "pix_hongguang_mini_ev_driver_msgs/vhiclemodecommand_100.h"
#include "pix_hongguang_mini_ev_driver_msgs/throttlecommand_101.h"
#include "pix_hongguang_mini_ev_driver_msgs/gearcommand_104.h"
#include "pix_hongguang_mini_ev_driver_msgs/steeringcomand_314.h"
#include "pix_hongguang_mini_ev_driver_msgs/brakecommand_364.h"



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include "vhiclemodecommand_100.hpp"
#include "throttlecommand_101.hpp"
#include "gearcommand_104.hpp"
#include "steeringcomand_314.hpp"
#include "brakecommand_364.hpp"



// global variable: msg
/*
Example:
static can_msgs::Frame can_brake;
static pix_driver_msgs::BrakeCommand brake_command_msg;

static can_msgs::Frame can_protocols["name"].split("_",1)[0];
static pix_driver_msgs::protocols["name"] protocols["name"]_msg
*/
static pix_hongguang_mini_ev_driver_msgs::vhiclemodecommand_100 vhiclemodecommand_100_msg;
static pix_hongguang_mini_ev_driver_msgs::throttlecommand_101 throttlecommand_101_msg;
static pix_hongguang_mini_ev_driver_msgs::gearcommand_104 gearcommand_104_msg;
static pix_hongguang_mini_ev_driver_msgs::steeringcomand_314 steeringcomand_314_msg;
static pix_hongguang_mini_ev_driver_msgs::brakecommand_364 brakecommand_364_msg;

static can_msgs::Frame can_vhiclemodecommand_100;
static can_msgs::Frame can_throttlecommand_101;
static can_msgs::Frame can_gearcommand_104;
static can_msgs::Frame can_steeringcomand_314;
static can_msgs::Frame can_brakecommand_364;


// global variable:  send entity
// Example: static Brakecommand101 brake_command;
// static protocols["name"] protocols["name"]_entity;
static Vhiclemodecommand100 vhiclemodecommand_100_entity;
static Throttlecommand101 throttlecommand_101_entity;
static Gearcommand104 gearcommand_104_entity;
static Steeringcomand314 steeringcomand_314_entity;
static Brakecommand364 brakecommand_364_entity;



// global variable: Enable and send time stamp
/*
Example:
static bool brake_enable, gear_enable, park_enable, steer_enable, throttle_enable;
static int brake_prev_t=0, gear_prev_t=0, park_prev_t=0, steer_prev_t=0, throttle_prev_t=0;

protocols["name"].split("_",1)[0]_enable, ...
protocols["name"].split("_",1)[0]_prev_t=0, ...
*/
static bool vhiclemodecommand_100_enable, throttlecommand_101_enable, gearcommand_104_enable, steeringcomand_314_enable, brakecommand_364_enable;
static int vhiclemodecommand_100_prev_t=0, throttlecommand_101_prev_t=0, gearcommand_104_prev_t=0, steeringcomand_314_prev_t=0, brakecommand_364_prev_t=0;


static int time_diff = 100000000;
static ros::Publisher pub_can;


// define send canID callback function

static void vhiclemodecommand_100_callback(const pix_hongguang_mini_ev_driver_msgs::vhiclemodecommand_100 &msg)
{
    
    vhiclemodecommand_100_entity.Reset();
    vhiclemodecommand_100_msg = msg;
    can_vhiclemodecommand_100.header.stamp = ros::Time::now();
    can_vhiclemodecommand_100.dlc = 8;
    vhiclemodecommand_100_entity.UpdateData(
        vhiclemodecommand_100_msg.AutoCtrlEna,
	vhiclemodecommand_100_msg.ModeCtrlCnt,
	vhiclemodecommand_100_msg.ModeCtrlCks
    );
    can_vhiclemodecommand_100.id = vhiclemodecommand_100_entity.ID;
    can_vhiclemodecommand_100.is_extended= false;
    uint8_t *A;
    A = vhiclemodecommand_100_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_vhiclemodecommand_100.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_vhiclemodecommand_100.header.stamp.toNSec();
    vhiclemodecommand_100_prev_t = t_nsec;
}

    
static void throttlecommand_101_callback(const pix_hongguang_mini_ev_driver_msgs::throttlecommand_101 &msg)
{
    
    throttlecommand_101_entity.Reset();
    throttlecommand_101_msg = msg;
    can_throttlecommand_101.header.stamp = ros::Time::now();
    can_throttlecommand_101.dlc = 8;
    throttlecommand_101_entity.UpdateData(
        throttlecommand_101_msg.AccPedCmd,
	throttlecommand_101_msg.AccCtrlEna,
	throttlecommand_101_msg.AccPedInv,
	throttlecommand_101_msg.AccCtrlCnt,
	throttlecommand_101_msg.AccCtrlCks,
	throttlecommand_101_msg.AccTkoDis
    );
    can_throttlecommand_101.id = throttlecommand_101_entity.ID;
    can_throttlecommand_101.is_extended= false;
    uint8_t *A;
    A = throttlecommand_101_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_throttlecommand_101.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_throttlecommand_101.header.stamp.toNSec();
    throttlecommand_101_prev_t = t_nsec;
}

    
static void gearcommand_104_callback(const pix_hongguang_mini_ev_driver_msgs::gearcommand_104 &msg)
{
    
    gearcommand_104_entity.Reset();
    gearcommand_104_msg = msg;
    can_gearcommand_104.header.stamp = ros::Time::now();
    can_gearcommand_104.dlc = 8;
    gearcommand_104_entity.UpdateData(
        gearcommand_104_msg.WiperCmd,
	gearcommand_104_msg.HornCmd,
	gearcommand_104_msg.GearCmd,
	gearcommand_104_msg.GearCtrlEna,
	gearcommand_104_msg.TurnLightCmd,
	gearcommand_104_msg.BeamLightCmd,
	gearcommand_104_msg.GearCtrlCnt,
	gearcommand_104_msg.GearCtrlCks
    );
    can_gearcommand_104.id = gearcommand_104_entity.ID;
    can_gearcommand_104.is_extended= false;
    uint8_t *A;
    A = gearcommand_104_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_gearcommand_104.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_gearcommand_104.header.stamp.toNSec();
    gearcommand_104_prev_t = t_nsec;
}

    
static void steeringcomand_314_callback(const pix_hongguang_mini_ev_driver_msgs::steeringcomand_314 &msg)
{
    
    steeringcomand_314_entity.Reset();
    steeringcomand_314_msg = msg;
    can_steeringcomand_314.header.stamp = ros::Time::now();
    can_steeringcomand_314.dlc = 8;
    steeringcomand_314_entity.UpdateData(
        steeringcomand_314_msg.work_state,
	steeringcomand_314_msg.tar_angle,
	steeringcomand_314_msg.cail_sas
    );
    can_steeringcomand_314.id = steeringcomand_314_entity.ID;
    can_steeringcomand_314.is_extended= false;
    uint8_t *A;
    A = steeringcomand_314_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_steeringcomand_314.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_steeringcomand_314.header.stamp.toNSec();
    steeringcomand_314_prev_t = t_nsec;
}

    
static void brakecommand_364_callback(const pix_hongguang_mini_ev_driver_msgs::brakecommand_364 &msg)
{
    
    brakecommand_364_entity.Reset();
    brakecommand_364_msg = msg;
    can_brakecommand_364.header.stamp = ros::Time::now();
    can_brakecommand_364.dlc = 8;
    brakecommand_364_entity.UpdateData(
        brakecommand_364_msg.VCU_sts,
	brakecommand_364_msg.VCU_SelfStudy,
	brakecommand_364_msg.VCU_KeySt,
	brakecommand_364_msg.VCU_ExtBrkpressure_V,
	brakecommand_364_msg.VCU_ExtBrkpressure,
	brakecommand_364_msg.VCU_EBS_StModeReq,
	brakecommand_364_msg.VCU_EBS_ctRoll,
	brakecommand_364_msg.VCU_DrvMode
    );
    can_brakecommand_364.id = brakecommand_364_entity.ID;
    can_brakecommand_364.is_extended= false;
    uint8_t *A;
    A = brakecommand_364_entity.get_data();
    for(unsigned int i=0;i<8;i++)
    {
        can_brakecommand_364.data[i] = *A;
        A += 1;
    }
    int t_nsec = 0;
    t_nsec = can_brakecommand_364.header.stamp.toNSec();
    brakecommand_364_prev_t = t_nsec;
}

    


//  define timer callback function
void timer_callback(const ros::TimerEvent &te)
{
    int now;
    now = ros::Time::now().toNSec();
    /*Example: 
    // brake
    if(now-brake_prev_t>time_diff)
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
    
    // vhiclemodecommand_100
    if(now - vhiclemodecommand_100_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_vhiclemodecommand_100.id = vhiclemodecommand_100_entity.ID;
            can_vhiclemodecommand_100.data[i] = 0;
        }
        pub_can.publish(can_vhiclemodecommand_100);
    }
    else{
        pub_can.publish(can_vhiclemodecommand_100);
    }
    
    // throttlecommand_101
    if(now - throttlecommand_101_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_throttlecommand_101.id = throttlecommand_101_entity.ID;
            can_throttlecommand_101.data[i] = 0;
        }
        pub_can.publish(can_throttlecommand_101);
    }
    else{
        pub_can.publish(can_throttlecommand_101);
    }
    
    // gearcommand_104
    if(now - gearcommand_104_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_gearcommand_104.id = gearcommand_104_entity.ID;
            can_gearcommand_104.data[i] = 0;
        }
        pub_can.publish(can_gearcommand_104);
    }
    else{
        pub_can.publish(can_gearcommand_104);
    }
    
    // steeringcomand_314
    if(now - steeringcomand_314_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_steeringcomand_314.id = steeringcomand_314_entity.ID;
            can_steeringcomand_314.data[i] = 0;
        }
        pub_can.publish(can_steeringcomand_314);
    }
    else{
        pub_can.publish(can_steeringcomand_314);
    }
    
    // brakecommand_364
    if(now - brakecommand_364_prev_t>time_diff)
    {
        for(uint i=0; i<8; i++)
        {   
            can_brakecommand_364.id = brakecommand_364_entity.ID;
            can_brakecommand_364.data[i] = 0;
        }
        pub_can.publish(can_brakecommand_364);
    }
    else{
        pub_can.publish(can_brakecommand_364);
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_hongguang_mini_ev_driver_command_node");
    ros::NodeHandle nh;

    // creat ros Subscriber
    // Example: ros::Subscriber sub_brake = nh.subscribe("/pix/brake_command", 1, brake_callback);
    ros::Subscriber sub_vhiclemodecommand_100 = nh.subscribe("/pix_hongguang_mini_ev/vhiclemodecommand_100", 1, vhiclemodecommand_100_callback);
	ros::Subscriber sub_throttlecommand_101 = nh.subscribe("/pix_hongguang_mini_ev/throttlecommand_101", 1, throttlecommand_101_callback);
	ros::Subscriber sub_gearcommand_104 = nh.subscribe("/pix_hongguang_mini_ev/gearcommand_104", 1, gearcommand_104_callback);
	ros::Subscriber sub_steeringcomand_314 = nh.subscribe("/pix_hongguang_mini_ev/steeringcomand_314", 1, steeringcomand_314_callback);
	ros::Subscriber sub_brakecommand_364 = nh.subscribe("/pix_hongguang_mini_ev/brakecommand_364", 1, brakecommand_364_callback);
	
    //  creat ros publisher
    pub_can = nh.advertise<can_msgs::Frame>("/sent_messages", 10, false);

    ros::Timer set_speed = nh.createTimer(ros::Duration(1/50.0), timer_callback);
    ros::spin();

    return 0;

}