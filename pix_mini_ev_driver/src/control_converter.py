#!/usr/bin/env python
#coding=utf-8

import math
import rospy
#-------------------------------eps_receive_314/转向, sendx_364/制动, msg100_100/模式, msg101_101/油门, msg104_104/档位, msg204_204/档位反馈, receive_289/制动反馈
from pix_mini_ev_driver_msgs.msg import eps_receive_314, sendx_364, msg100_100, msg101_101, msg104_104, msg204_204, receive_289
from autoware_vehicle_msgs.msg import RawVehicleCommand

steering_factor = 500.0/(math.pi*(32.0/180))

class ControlConverter:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0
    
        self.gear_state = 0
        
        self.sub_raw_command = rospy.Subscriber('/vehicle/raw_vehicle_cmd', RawVehicleCommand, self.raw_command_callback)
        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', msg204_204, self.gear_report_callback)


        self.pub_throttle = rospy.Publisher('/pix/throttle_command', msg101_101, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command', sendx_364, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steering_command', eps_receive_314, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command', msg104_104, queue_size=10)

        self.throttle_msg = msg101_101()
        self.brake_msg = sendx_364()
        self.steer_msg = eps_receive_314()
        self.gear_msg = msg104_104()
        self.vehicle_msg = msg100_100() 
        self.gear_report_msg = msg204_204()    
        self.brake_report_msg = receive_289()

    def gear_report_callback(self, msg):
        # self.gear_state = msg.Gear_Actual 
        self.gear_state = msg.GearCmd
    
    def raw_command_callback(self, msg):
        
        self.throttle = msg.control.throttle
        self.brake = msg.control.brake
        self.steer = msg.control.steering_angle
        self.gear = msg.shift.data
        stamp = rospy.Time.now()
        
        # 刹车功能设计
        # self.brake_msg.aeb_en_ctrl = rospy.get_param("aeb_en_ctrl", 0)
        self.brake_msg.header.stamp = stamp
        
        self.brake_msg.VCU_ExtBrkpressure = self.brake*100.0
        # self.brake_msg.Brake_EnCtrl = 1

        self.steer_msg.header.stamp = stamp
        self.steer_msg.work_state = True
        self.steer_msg.tar_angle = self.steer * steering_factor
        # self.steer_msg.Steer_AngleSpeed = 250
        self.pub_steer.publish(self.steer_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.GearCtrlEna = True
        # :2=R档,3=N档,4=D档,其他保留。
        # self.gear_msg.GearCmd
        if(self.gear==0):
            self.gear_msg.GearCmd = 0
        elif(self.gear==1):
            self.gear_msg.GearCmd = 0
        elif(self.gear==2):
            self.gear_msg.GearCmd = 2
        elif(self.gear==3):
            self.gear_msg.GearCmd = 3
        elif(self.gear==4):
            self.gear_msg.GearCmd = 4
        
        self.pub_gear.publish(self.gear_msg)

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.AccPedCmd = self.throttle*100.0
        self.throttle_msg.AccCtrlEna = True

        # 当档位不一致时，
        # if(self.gear_state!=self.gear):
        #     self.throttle_msg.Dirve_ThrottlePedalTarget = 0
        #     self.brake_msg.Brake_Pedal_Target = 30
        
        self.pub_throttle.publish(self.throttle_msg)

        self.pub_brake.publish(self.brake_msg)


if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()