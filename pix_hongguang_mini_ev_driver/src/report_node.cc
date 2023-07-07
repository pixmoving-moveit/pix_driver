#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "std_msgs/Header.h"

// name = 
// # include "pix pix_driver_msgs/{name}.h"
#include "pix_hongguang_mini_ev_driver_msgs/steeringreport_18f.h"
#include "pix_hongguang_mini_ev_driver_msgs/vhiclemodereport_200.h"
#include "pix_hongguang_mini_ev_driver_msgs/throttlereport_201.h"
#include "pix_hongguang_mini_ev_driver_msgs/gearreport_204.h"
#include "pix_hongguang_mini_ev_driver_msgs/brakereport_289.h"


// # include "{name}.hpp"
#include "steeringreport_18f.hpp"
#include "vhiclemodereport_200.hpp"
#include "throttlereport_201.hpp"
#include "gearreport_204.hpp"
#include "brakereport_289.hpp"


static can_msgs::Frame can_frame_msg;

// static ros::Publisher pub_{can_name};
static ros::Publisher pub_steeringreport;
static ros::Publisher pub_vhiclemodereport;
static ros::Publisher pub_throttlereport;
static ros::Publisher pub_gearreport;
static ros::Publisher pub_brakereport;


// static pix_driver_msgs::{name} {name}_msg;
static pix_hongguang_mini_ev_driver_msgs::steeringreport_18f steeringreport_18f_msg;
static pix_hongguang_mini_ev_driver_msgs::vhiclemodereport_200 vhiclemodereport_200_msg;
static pix_hongguang_mini_ev_driver_msgs::throttlereport_201 throttlereport_201_msg;
static pix_hongguang_mini_ev_driver_msgs::gearreport_204 gearreport_204_msg;
static pix_hongguang_mini_ev_driver_msgs::brakereport_289 brakereport_289_msg;


// static {name}.replace('_', '').capitalize()  {name}_entity
static Steeringreport18f  steeringreport_18f_entity;
static Vhiclemodereport200  vhiclemodereport_200_entity;
static Throttlereport201  throttlereport_201_entity;
static Gearreport204  gearreport_204_entity;
static Brakereport289  brakereport_289_entity;



// callback func
static void can_callback(const can_msgs::Frame &msg)
{
    can_frame_msg = msg;
    std_msgs::Header header;
    header.frame_id = "pix";
    header.stamp = can_frame_msg.header.stamp;
    
    if(can_frame_msg.id==steeringreport_18f_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        steeringreport_18f_entity.update_bytes(byte_temp);
        steeringreport_18f_entity.Parse();
        steeringreport_18f_msg.header = header;
		steeringreport_18f_msg.XJF = steeringreport_18f_entity.xjf;
		steeringreport_18f_msg.real_current = steeringreport_18f_entity.real_current;
		steeringreport_18f_msg.real_angle = steeringreport_18f_entity.real_angle;
		steeringreport_18f_msg.ECU_temputer = steeringreport_18f_entity.ecu_temputer;
		steeringreport_18f_msg.ECU_state = steeringreport_18f_entity.ecu_state;

        pub_steeringreport.publish(steeringreport_18f_msg);
    }
    
    if(can_frame_msg.id==vhiclemodereport_200_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        vhiclemodereport_200_entity.update_bytes(byte_temp);
        vhiclemodereport_200_entity.Parse();
        vhiclemodereport_200_msg.header = header;
		vhiclemodereport_200_msg.AutoCtrlStat = vhiclemodereport_200_entity.autoctrlstat;
		vhiclemodereport_200_msg.ModeStatCnt = vhiclemodereport_200_entity.modestatcnt;
		vhiclemodereport_200_msg.ModeStatCks = vhiclemodereport_200_entity.modestatcks;

        pub_vhiclemodereport.publish(vhiclemodereport_200_msg);
    }
    
    if(can_frame_msg.id==throttlereport_201_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        throttlereport_201_entity.update_bytes(byte_temp);
        throttlereport_201_entity.Parse();
        throttlereport_201_msg.header = header;
		throttlereport_201_msg.AccPedAct = throttlereport_201_entity.accpedact;
		throttlereport_201_msg.AccPedCmd = throttlereport_201_entity.accpedcmd;
		throttlereport_201_msg.AccPedExe = throttlereport_201_entity.accpedexe;
		throttlereport_201_msg.AccCtrlStat = throttlereport_201_entity.accctrlstat;
		throttlereport_201_msg.AccStatCnt = throttlereport_201_entity.accstatcnt;
		throttlereport_201_msg.AccStatCks = throttlereport_201_entity.accstatcks;

        pub_throttlereport.publish(throttlereport_201_msg);
    }
    
    if(can_frame_msg.id==gearreport_204_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        gearreport_204_entity.update_bytes(byte_temp);
        gearreport_204_entity.Parse();
        gearreport_204_msg.header = header;
		gearreport_204_msg.VehSpd = gearreport_204_entity.vehspd;
		gearreport_204_msg.WiperCmd = gearreport_204_entity.wipercmd;
		gearreport_204_msg.WiperAct = gearreport_204_entity.wiperact;
		gearreport_204_msg.HornCmd = gearreport_204_entity.horncmd;
		gearreport_204_msg.HornStatus = gearreport_204_entity.hornstatus;
		gearreport_204_msg.GearAct = gearreport_204_entity.gearact;
		gearreport_204_msg.GearCmd = gearreport_204_entity.gearcmd;
		gearreport_204_msg.TurnLightCmd = gearreport_204_entity.turnlightcmd;
		gearreport_204_msg.TurnLightAct = gearreport_204_entity.turnlightact;
		gearreport_204_msg.BeamLightAct = gearreport_204_entity.beamlightact;
		gearreport_204_msg.BeamLightCmd = gearreport_204_entity.beamlightcmd;
		gearreport_204_msg.GearStatCnt = gearreport_204_entity.gearstatcnt;
		gearreport_204_msg.GearStatCks = gearreport_204_entity.gearstatcks;

        pub_gearreport.publish(gearreport_204_msg);
    }
    
    if(can_frame_msg.id==brakereport_289_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        brakereport_289_entity.update_bytes(byte_temp);
        brakereport_289_entity.Parse();
        brakereport_289_msg.header = header;
		brakereport_289_msg.EBS_MotAglFbk = brakereport_289_entity.ebs_motaglfbk;
		brakereport_289_msg.EBS_WarningLight = brakereport_289_entity.ebs_warninglight;
		brakereport_289_msg.EBS_StMode = brakereport_289_entity.ebs_stmode;
		brakereport_289_msg.EBS_PresFbk = brakereport_289_entity.ebs_presfbk;
		brakereport_289_msg.EBS_Fault_Info = brakereport_289_entity.ebs_fault_info;
		brakereport_289_msg.EBS_ExtBrkPrio = brakereport_289_entity.ebs_extbrkprio;
		brakereport_289_msg.EBS_DrvrInteraction = brakereport_289_entity.ebs_drvrinteraction;
		brakereport_289_msg.EBS_ctRoll = brakereport_289_entity.ebs_ctroll;
		brakereport_289_msg.EBS_BrkPedalApplied_V = brakereport_289_entity.ebs_brkpedalapplied_v;
		brakereport_289_msg.EBS_BrkPedalApplied = brakereport_289_entity.ebs_brkpedalapplied;
		brakereport_289_msg.EBS_Brakelight = brakereport_289_entity.ebs_brakelight;

        pub_brakereport.publish(brakereport_289_msg);
    }
    
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_hongguang_mini_ev_driver_report_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/received_messages", 1, can_callback);

    // pub_{can_name} = nh.advertise<pix_driver_msgs::{name}>("/pix/{can_name}_report", 1, true);
    pub_steeringreport = nh.advertise<pix_hongguang_mini_ev_driver_msgs::steeringreport_18f>("/pix_hongguang_mini_ev/steeringreport", 1, true);
	pub_vhiclemodereport = nh.advertise<pix_hongguang_mini_ev_driver_msgs::vhiclemodereport_200>("/pix_hongguang_mini_ev/vhiclemodereport", 1, true);
	pub_throttlereport = nh.advertise<pix_hongguang_mini_ev_driver_msgs::throttlereport_201>("/pix_hongguang_mini_ev/throttlereport", 1, true);
	pub_gearreport = nh.advertise<pix_hongguang_mini_ev_driver_msgs::gearreport_204>("/pix_hongguang_mini_ev/gearreport", 1, true);
	pub_brakereport = nh.advertise<pix_hongguang_mini_ev_driver_msgs::brakereport_289>("/pix_hongguang_mini_ev/brakereport", 1, true);
	
    // add another publisher

    ros::spin();
    return 0;

}
