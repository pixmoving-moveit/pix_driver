#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "std_msgs/Header.h"

// name = 
// # include "pix pix_mini_ev_driver_msgs/{name}.h"
#include "pix_mini_ev_driver_msgs/eps_send2_18e.h"
#include "pix_mini_ev_driver_msgs/eps_send_18f.h"
#include "pix_mini_ev_driver_msgs/msg200_200.h"
#include "pix_mini_ev_driver_msgs/msg201_201.h"
#include "pix_mini_ev_driver_msgs/msg204_204.h"
#include "pix_mini_ev_driver_msgs/receive_289.h"
#include "pix_mini_ev_driver_msgs/eps_receive_314.h"
#include "pix_mini_ev_driver_msgs/eps_receive2_315.h"
#include "pix_mini_ev_driver_msgs/sendx_364.h"


// # include "{name}.hpp"
#include "eps_send2_18e.hpp"
#include "eps_send_18f.hpp"
#include "msg200_200.hpp"
#include "msg201_201.hpp"
#include "msg204_204.hpp"
#include "receive_289.hpp"
#include "eps_receive_314.hpp"
#include "eps_receive2_315.hpp"
#include "sendx_364.hpp"


static can_msgs::Frame can_frame_msg;

// static ros::Publisher pub_{can_name};
static ros::Publisher pub_eps_send2;
static ros::Publisher pub_eps_send;
static ros::Publisher pub_msg200;
static ros::Publisher pub_msg201;
static ros::Publisher pub_msg204;
static ros::Publisher pub_receive;
static ros::Publisher pub_eps_receive;
static ros::Publisher pub_eps_receive2;
static ros::Publisher pub_sendx;


// static pix_mini_ev_driver_msgs::{name} {name}_msg;
static pix_mini_ev_driver_msgs::eps_send2_18e eps_send2_18e_msg;
static pix_mini_ev_driver_msgs::eps_send_18f eps_send_18f_msg;
static pix_mini_ev_driver_msgs::msg200_200 msg200_200_msg;
static pix_mini_ev_driver_msgs::msg201_201 msg201_201_msg;
static pix_mini_ev_driver_msgs::msg204_204 msg204_204_msg;
static pix_mini_ev_driver_msgs::receive_289 receive_289_msg;
static pix_mini_ev_driver_msgs::eps_receive_314 eps_receive_314_msg;
static pix_mini_ev_driver_msgs::eps_receive2_315 eps_receive2_315_msg;
static pix_mini_ev_driver_msgs::sendx_364 sendx_364_msg;


// static {name}.replace('_', '').capitalize()  {name}_entity
static Epssend218e  eps_send2_18e_entity;
static Epssend18f  eps_send_18f_entity;
static Msg200200  msg200_200_entity;
static Msg201201  msg201_201_entity;
static Msg204204  msg204_204_entity;
static Receive289  receive_289_entity;
static Epsreceive314  eps_receive_314_entity;
static Epsreceive2315  eps_receive2_315_entity;
static Sendx364  sendx_364_entity;



// callback func
static void can_callback(const can_msgs::Frame &msg)
{
    can_frame_msg = msg;
    std_msgs::Header header;
    header.frame_id = "pix";
    header.stamp = can_frame_msg.header.stamp;
    
    if(can_frame_msg.id==eps_send2_18e_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        eps_send2_18e_entity.update_bytes(byte_temp);
        eps_send2_18e_entity.Parse();
        eps_send2_18e_msg.header = header;
		eps_send2_18e_msg.work_mode = eps_send2_18e_entity.work_mode;
		eps_send2_18e_msg.Torger_value = eps_send2_18e_entity.torger_value;
		eps_send2_18e_msg.Tar_current = eps_send2_18e_entity.tar_current;
		eps_send2_18e_msg.Steer_speed = eps_send2_18e_entity.steer_speed;

        pub_eps_send2.publish(eps_send2_18e_msg);
    }
    
    if(can_frame_msg.id==eps_send_18f_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        eps_send_18f_entity.update_bytes(byte_temp);
        eps_send_18f_entity.Parse();
        eps_send_18f_msg.header = header;
		eps_send_18f_msg.XJF = eps_send_18f_entity.xjf;
		eps_send_18f_msg.real_current = eps_send_18f_entity.real_current;
		eps_send_18f_msg.real_angle = eps_send_18f_entity.real_angle;
		eps_send_18f_msg.ECU_temputer = eps_send_18f_entity.ecu_temputer;
		eps_send_18f_msg.ECU_state = eps_send_18f_entity.ecu_state;

        pub_eps_send.publish(eps_send_18f_msg);
    }
    
    if(can_frame_msg.id==msg200_200_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        msg200_200_entity.update_bytes(byte_temp);
        msg200_200_entity.Parse();
        msg200_200_msg.header = header;
		msg200_200_msg.AutoCtrlStat = msg200_200_entity.autoctrlstat;
		msg200_200_msg.ModeStatCnt = msg200_200_entity.modestatcnt;
		msg200_200_msg.ModeStatCks = msg200_200_entity.modestatcks;

        pub_msg200.publish(msg200_200_msg);
    }
    
    if(can_frame_msg.id==msg201_201_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        msg201_201_entity.update_bytes(byte_temp);
        msg201_201_entity.Parse();
        msg201_201_msg.header = header;
		msg201_201_msg.AccPedAct = msg201_201_entity.accpedact;
		msg201_201_msg.AccPedCmd = msg201_201_entity.accpedcmd;
		msg201_201_msg.AccPedExe = msg201_201_entity.accpedexe;
		msg201_201_msg.AccCtrlStat = msg201_201_entity.accctrlstat;
		msg201_201_msg.AccStatCnt = msg201_201_entity.accstatcnt;
		msg201_201_msg.AccStatCks = msg201_201_entity.accstatcks;

        pub_msg201.publish(msg201_201_msg);
    }
    
    if(can_frame_msg.id==msg204_204_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        msg204_204_entity.update_bytes(byte_temp);
        msg204_204_entity.Parse();
        msg204_204_msg.header = header;
		msg204_204_msg.VehSpd = msg204_204_entity.vehspd;
		msg204_204_msg.WiperCmd = msg204_204_entity.wipercmd;
		msg204_204_msg.WiperAct = msg204_204_entity.wiperact;
		msg204_204_msg.HornCmd = msg204_204_entity.horncmd;
		msg204_204_msg.HornStatus = msg204_204_entity.hornstatus;
		msg204_204_msg.GearAct = msg204_204_entity.gearact;
		msg204_204_msg.GearCmd = msg204_204_entity.gearcmd;
		msg204_204_msg.TurnLightCmd = msg204_204_entity.turnlightcmd;
		msg204_204_msg.TurnLightAct = msg204_204_entity.turnlightact;
		msg204_204_msg.BeamLightAct = msg204_204_entity.beamlightact;
		msg204_204_msg.BeamLightCmd = msg204_204_entity.beamlightcmd;
		msg204_204_msg.GearStatCnt = msg204_204_entity.gearstatcnt;
		msg204_204_msg.GearStatCks = msg204_204_entity.gearstatcks;

        pub_msg204.publish(msg204_204_msg);
    }
    
    if(can_frame_msg.id==receive_289_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        receive_289_entity.update_bytes(byte_temp);
        receive_289_entity.Parse();
        receive_289_msg.header = header;
		receive_289_msg.EBS_MotAglFbk = receive_289_entity.ebs_motaglfbk;
		receive_289_msg.EBS_WarningLight = receive_289_entity.ebs_warninglight;
		receive_289_msg.EBS_StMode = receive_289_entity.ebs_stmode;
		receive_289_msg.EBS_PresFbk = receive_289_entity.ebs_presfbk;
		receive_289_msg.EBS_Fault_Info = receive_289_entity.ebs_fault_info;
		receive_289_msg.EBS_ExtBrkPrio = receive_289_entity.ebs_extbrkprio;
		receive_289_msg.EBS_DrvrInteraction = receive_289_entity.ebs_drvrinteraction;
		receive_289_msg.EBS_ctRoll = receive_289_entity.ebs_ctroll;
		receive_289_msg.EBS_BrkPedalApplied_V = receive_289_entity.ebs_brkpedalapplied_v;
		receive_289_msg.EBS_BrkPedalApplied = receive_289_entity.ebs_brkpedalapplied;
		receive_289_msg.EBS_Brakelight = receive_289_entity.ebs_brakelight;

        pub_receive.publish(receive_289_msg);
    }
    
    if(can_frame_msg.id==eps_receive_314_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        eps_receive_314_entity.update_bytes(byte_temp);
        eps_receive_314_entity.Parse();
        eps_receive_314_msg.header = header;
		eps_receive_314_msg.work_state = eps_receive_314_entity.work_state;
		eps_receive_314_msg.tar_angle = eps_receive_314_entity.tar_angle;
		eps_receive_314_msg.cail_sas = eps_receive_314_entity.cail_sas;

        pub_eps_receive.publish(eps_receive_314_msg);
    }
    
    if(can_frame_msg.id==eps_receive2_315_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        eps_receive2_315_entity.update_bytes(byte_temp);
        eps_receive2_315_entity.Parse();
        eps_receive2_315_msg.header = header;
		eps_receive2_315_msg.Vech_value = eps_receive2_315_entity.vech_value;
		eps_receive2_315_msg.Vech_enable = eps_receive2_315_entity.vech_enable;

        pub_eps_receive2.publish(eps_receive2_315_msg);
    }
    
    if(can_frame_msg.id==sendx_364_entity.ID)
    {
        uint8_t byte_temp[8];
        for(uint i=0;i<8;i++)
        {
        byte_temp[i] = can_frame_msg.data[i];
        }
        sendx_364_entity.update_bytes(byte_temp);
        sendx_364_entity.Parse();
        sendx_364_msg.header = header;
		sendx_364_msg.VCU_sts = sendx_364_entity.vcu_sts;
		sendx_364_msg.VCU_SelfStudy = sendx_364_entity.vcu_selfstudy;
		sendx_364_msg.VCU_KeySt = sendx_364_entity.vcu_keyst;
		sendx_364_msg.VCU_ExtBrkpressure_V = sendx_364_entity.vcu_extbrkpressure_v;
		sendx_364_msg.VCU_ExtBrkpressure = sendx_364_entity.vcu_extbrkpressure;
		sendx_364_msg.VCU_EBS_StModeReq = sendx_364_entity.vcu_ebs_stmodereq;
		sendx_364_msg.VCU_EBS_ctRoll = sendx_364_entity.vcu_ebs_ctroll;
		sendx_364_msg.VCU_DrvMode = sendx_364_entity.vcu_drvmode;

        pub_sendx.publish(sendx_364_msg);
    }
    
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pix_mini_ev_driver_report_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/received_messages", 1, can_callback);

    // pub_{can_name} = nh.advertise<pix_mini_ev_driver_msgs::{name}>("/pix/{can_name}_report", 1, true);
    pub_eps_send2 = nh.advertise<pix_mini_ev_driver_msgs::eps_send2_18e>("/pix/eps_send2", 1, true);
	pub_eps_send = nh.advertise<pix_mini_ev_driver_msgs::eps_send_18f>("/pix/eps_send", 1, true);
	pub_msg200 = nh.advertise<pix_mini_ev_driver_msgs::msg200_200>("/pix/msg200", 1, true);
	pub_msg201 = nh.advertise<pix_mini_ev_driver_msgs::msg201_201>("/pix/msg201", 1, true);
	pub_msg204 = nh.advertise<pix_mini_ev_driver_msgs::msg204_204>("/pix/msg204", 1, true);
	pub_receive = nh.advertise<pix_mini_ev_driver_msgs::receive_289>("/pix/receive", 1, true);
	pub_eps_receive = nh.advertise<pix_mini_ev_driver_msgs::eps_receive_314>("/pix/eps_receive", 1, true);
	pub_eps_receive2 = nh.advertise<pix_mini_ev_driver_msgs::eps_receive2_315>("/pix/eps_receive2", 1, true);
	pub_sendx = nh.advertise<pix_mini_ev_driver_msgs::sendx_364>("/pix/sendx", 1, true);
	
    // add another publisher

    ros::spin();
    return 0;

}
