#!/usr/bin/env python
#coding=utf-8

import rospy
from pix_hongguang_mini_ev_driver_msgs.msg import vhiclemodecommand_100, throttlecommand_101, gearcommand_104, steeringcomand_314, brakecommand_364, steeringreport_18f, vhiclemodereport_200, throttlereport_201, gearreport_204, brakereport_289
from std_msgs.msg import Bool

class ControlConverter:
    def __init__(self):

        
        self.su_steer_report = rospy.Subscriber('/pix/steering_report', steeringreport_18f, self.steer_report_callback)        
        
        self.steer_msg_data_ = steeringreport_18f()


        self.pub_throttle = rospy.Publisher('/pix/throttle_command_101', throttlecommand_101, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command_364', brakecommand_364, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steering_comand_314', steeringcomand_314, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command_104', gearcommand_104, queue_size=10)
        self.pub_vehicle = rospy.Publisher('/pix/vhicle_mode_command_100', vhiclemodecommand_100, queue_size=10)

        self.throttle_msg = throttlecommand_101()
        self.brake_msg = brakecommand_364()
        self.steer_msg = steeringcomand_314()
        self.gear_msg = gearcommand_104()
        self.vehicle_msg = vhiclemodecommand_100() 
       
    # 使得车辆退出自动驾驶模式
    def steer_report_callback(self,msg):

        stamp = rospy.Time.now()
        
        if(msg.ECU_state != 1):
            self.vehicle_msg.AutoCtrlEna = 0       
        else:
            self.vehicle_msg.AutoCtrlEna = 1

        self.vehicle_msg.header.stamp = stamp
        
        self.brake_msg.header.stamp = stamp
        self.brake_msg.VCU_ExtBrkpressure = 0

        self.steer_msg.header.stamp = stamp
        self.steer_msg.work_state = False
        self.steer_msg.tar_angle = 0 

        self.gear_msg.header.stamp = stamp
        self.gear_msg.GearCtrlEna = False
        self.gear_msg.GearCmd = msg.GearCmd

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.AccCtrlEna = False
        self.throttle_msg.AccPedCmd = 0
        self.throttle_msg.AccPedInv = 0
      
        self.pub_vehicle.publish(self.vehicle_msg)    #100
        self.pub_steer.publish(self.steer_msg)        #314
        self.pub_gear.publish(self.gear_msg)          #104
        self.pub_throttle.publish(self.throttle_msg)  #101
        self.pub_brake.publish(self.brake_msg)        #364



if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()