#!/usr/bin/env python
#coding=utf-8

import math
import rospy
from pix_hongguang_mini_ev_driver_msgs.msg import vhiclemodecommand_100, throttlecommand_101, gearcommand_104, steeringcomand_314, brakecommand_364, steeringreport_18f, vhiclemodereport_200, throttlereport_201, gearreport_204, brakereport_289
from std_msgs.msg import Bool
from autoware_vehicle_msgs.msg import RawVehicleCommand

steering_factor = 540.0/(math.pi*(32.0/180))

class ControlConverter:
    def __init__(self):
        self.sub_raw_command = rospy.Subscriber('/vehicle/raw_vehicle_cmd', RawVehicleCommand, self.raw_command_callback)

        self.sub_vehicle_steer_mode_report = rospy.Subscriber('/pix/vhicle_mode_report', vhiclemodereport_200, self.vehicle_mode_report_callback)
        self.sub_brake_report = rospy.Subscriber('/pix/brake_report', brakereport_289, self.brake_report_callback)
        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', gearreport_204, self.gear_report_callback)
        
        self.su_steer_report = rospy.Subscriber('/pix/steering_report', steeringreport_18f, self.steer_report_callback)

        self.sub_vehicle_engage = rospy.Subscriber('/pix/vehicle_engage', Bool, self.vehicle_engage_mode, queue_size=10)
        
        self.vehicle_mode_ = False
        self.gear_msg_data_ = 0
        self.brake_msg_data_ = False
        self.vehicle_msg_data_ = False 
        self.steer_msg_data_ = False

        self.pub_throttle = rospy.Publisher('/pix/throttle_command_101', throttlecommand_101, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command_364', brakecommand_364, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steering_comand_314', steeringcomand_314, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command_104', gearcommand_104, queue_size=10)
        self.pub_vehicle = rospy.Publisher('/pix/vhicle_mode_command_100', vhiclemodecommand_100, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.01),self.time_callback)

        self.throttle_msg = throttlecommand_101()
        self.brake_msg = brakecommand_364()
        self.steer_msg = steeringcomand_314()
        self.gear_msg = gearcommand_104()
        self.vehicle_msg = vhiclemodecommand_100() 

    def steer_report_callback(self,msg):
        if(msg.ECU_state == 1):
            self.steer_msg_data_ = True
        else:
            self.steer_msg_data_ = False

    def vehicle_mode_report_callback(self,msg):
        if (msg.AutoCtrlStat == 1):
            self.vehicle_msg_data_ = True
        else:
            self.vehicle_msg_data_ = False

    def time_callback(self, event):
      # 进入自动驾驶模式
      if(self.vehicle_mode_):
        self.vehicle_msg.AutoCtrlEna = 1
      else:
        self.vehicle_msg.AutoCtrlEna = 0
      self.vehicle_msg.header.stamp = rospy.Time.now()
      self.pub_vehicle.publish(self.vehicle_msg) #ID 100

    def vehicle_engage_mode(self, msg):
        self.vehicle_mode_ = msg.data

    def brake_report_callback(self, msg):
        if(msg.EBS_PresFbk>0):
          self.brake_msg_data_ = True
        else:
          self.brake_msg_data_ = False

    def gear_report_callback(self, msg):
        self.gear_msg_data_ = msg.GearCmd

    def raw_command_callback(self, msg):
        # 获取autoware - 控制信息
        self.throttle = msg.control.throttle
        self.brake = msg.control.brake
        self.steer = msg.control.steering_angle
        self.gear = msg.shift.data

        stamp = rospy.Time.now()
        
        self.brake_msg.header.stamp = stamp
        self.brake_msg.VCU_ExtBrkpressure = self.brake

        self.steer_msg.header.stamp = stamp
        self.steer_msg.work_state = True
        self.steer_msg.tar_angle = self.steer * steering_factor

        self.gear_msg.header.stamp = stamp
        self.gear_msg.GearCtrlEna = True
        self.gear_msg.GearCmd = self.gear

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.AccCtrlEna = True
        self.throttle_msg.AccPedCmd = self.throttle
        self.throttle_msg.AccPedInv = self.throttle


        # 使得车辆进入自动驾驶模式
        if(self.vehicle_mode_ and (not self.vehicle_msg_data_)):
            self.throttle_msg.AccPedCmd = 0
            self.throttle_msg.AccPedInv = 0
            self.steer_msg.tar_angle = 0
            self.gear_msg.GearCmd = 3
            self.brake_msg.VCU_ExtBrkpressure = (125.0 * 0.2)        
        
        # 有刹车切换档位
            if(self.brake_msg_data_):
              self.gear_msg.GearCmd = self.gear


        self.pub_steer.publish(self.steer_msg)        #314
        self.pub_gear.publish(self.gear_msg)          #104
        self.pub_throttle.publish(self.throttle_msg)  #101
        self.pub_brake.publish(self.brake_msg)        #364

      


if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()