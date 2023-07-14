#!/usr/bin/env python
#coding=utf-8

import time
import rospy
from pix_hongguang_mini_ev_driver_msgs.msg import vhiclemodecommand_100, throttlecommand_101, gearcommand_104, steeringcomand_314, brakecommand_364, steeringreport_18f, vhiclemodereport_200, throttlereport_201, gearreport_204, brakereport_289
from autoware_vehicle_msgs.msg import RawVehicleCommand


class ControlConverter:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0   
        self.gear_state = 0

        self.vehiclemodel_msg = vhiclemodecommand_100() 
        self.throttle_msg = throttlecommand_101()
        self.brake_msg = brakecommand_364()
        self.steer_msg = steeringcomand_314()
        self.gear_msg = gearcommand_104()
    
        self.vehiclemodel_report_msg = vhiclemodereport_200() 
        self.throttle_report_msg = throttlereport_201()
        self.brake_report_msg = brakereport_289()
        self.steer_report_msg = steeringreport_18f()
        self.gear_report_msg = gearreport_204()             

    def gear_report_callback(self, msg):
        self.gear_state = msg.GearCmd
    
    def raw_command_callback(self):
        
        # self.throttle.cmd = msg.AccPedCmd
        # self.throttle.inv = msg.AccPedInv
        # self.brake = msg.VCU_ExtBrkpressure
        # self.steer = msg.tar_angle
        # self.gear_state = msg.GearCmd
        # self.vehicle_model = msg.AutoCtrlEna
        
        # 进入线控模式
        for _ in range(5): #----------循环发送5次，激活线控
        #线控使能
            #自驾使能
            self.vehiclemodel_msg.AutoCtrlEna = 1
            self.pub_vehicle.publish(self.vehiclemodel_msg)

            #油门初始化0
            self.throttle_msg.AccPedCmd = 0
            self.throttle_msg.AccPedInv = 0
            self.pub_throttle.publish(self.throttle_msg)

            #方向初始化0
            # self.steer_msg.tar_angle = 0
            self.steer_msg.tar_angle = 0
            self.pub_steer.publish(self.steer_msg)

            #档位N
            self.gear_msg.GearCmd = 3
            self.pub_gear.publish(self.gear_msg)

            self.brake_msg.VCU_ExtBrkpressure = 125*0.2
            self.pub_brake.publish(self.brake_msg) #---------- 制动


        rospy.sleep(0.1)  # 每次发送之间的间隔时间

        time.sleep(5)


        # 进行条件判断,判断是否已经制动
        if (self.brake_report_msg.EBS_PresFbk >= self.brake_msg.VCU_ExtBrkpressure * 0.8  & self.brake_report_msg.EBS_PresFbk <= self.brake_msg.VCU_ExtBrkpressure):
            # 开始换挡 2:R 3:N 4:D
            self.gear_msg.GearCmd = 4
            self.pub_gear.publish(self.gear_msg)
        
            time.sleep(1)

            # 当档位不一致时，
            if (self.gear_report_msg.GearCmd == self.gear_msg.GearCmd):
                self.brake_msg.VCU_ExtBrkpressure = 0
                self.pub_brake.publish(self.brake_msg) #---------- 松开制动
                time.sleep(1)

            # 当档位不一致时，
            elif(self.gear_report_msg.GearCmd != self.gear_msg.GearCmd):
                self.throttle_msg.AccPedCmd = 0
                self.brake_msg.VCU_ExtBrkpressure = 120 * 0.5       
                self.pub_throttle.publish(self.throttle_msg)
                self.pub_brake.publish(self.brake_msg)


if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()