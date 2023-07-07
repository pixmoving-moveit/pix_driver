#!/usr/bin/env python
#coding=utf-8

import math
import rospy
from pix_driver_msgs.msg import eps_receive_314, sendx_364, msg100_100, msg101_101, msg104_104, msg204_204, receive_289
#-------------------------------eps_receive_314/转向, sendx_364/制动, msg100_100/模式, msg101_101/油门, msg104_104/档位, msg204_204/档位反馈, receive_289/制动反馈
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
        self.pub_vehicle = rospy.Publisher('/pix/vehicle_mode_command', msg100_100, queue_size=10)

        self.throttle_msg = msg101_101()
        self.brake_msg = sendx_364()
        self.steer_msg = eps_receive_314()
        self.gear_msg = msg104_104()
        self.vehicle_msg = msg100_100() 
        self.gear_report_msg = msg204_204()    
        self.brake_report_msg = receive_289()

    def Line_control(self,msg):   #----------线控模式
        for _ in range(5):        #----------循环发送5次，激活线控

            self.throttle_msg.id = 101
            self.brake_msg.id = 364
            self.steer_msg.id = 314
            self.gear_msg.id = 104
            self.vehicle_msg.id = 100
            self.gear_report_msg.id = 204   
            self.brake_report_msg.id = 289

            self.throttle_msg.data = [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]   # 发布报文ID为101，数据内容为80 00 00 00 00 00 00 00
            self.brake_msg.data = [0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            self.steer_msg.data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            self.gear_msg.data = [0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]       # 82=R 83=N 84=D
            self.vehicle_msg.data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]    # 00工人 01线控

            self.pub_throttle.publish(self.throttle_msg)
            self.pub_steer.publish(self.steer_msg)
            self.pub_gear.publish(self.gear_msg)
            self.pub_vehicle.publish(self.vehicle_msg)

            rospy.sleep(0.1)  # 每次发送之间的间隔时间

        self.pub_brake.publish(self.brake_msg) #---------- 制动，并准备换挡

        # 接收到消息后进行条件判断
        if msg.id == 289:
            # 判断是否已经制动
            if msg.data[1] == 0x34:   
                # 已经5%的制动，0x34换为油门的踏板量5%左右。
                # 切换为D档位
                self.gear_msg.id = 104
                self.gear_msg.data = [0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                self.pub_gear.publish(self.gear_msg)
    
        # 在接收到消息后进行第二个条件判断
        if msg.id == 204:
            # 判断是否换挡成功
            if msg.data[0] == 0x44:
                # 如换挡成功，松开制动开始下一步操作。
                # 发送报文ID为364，内容为00 00 00 00 00 00 00 00，松开制动
                self.brake_msg.id = 364
                self.brake_msg.data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                self.pub_brake.publish(self.brake_msg)
                     

    def gear_report_callback(self, msg):
        # self.gear_state = msg.Gear_Actual 
        self.gear_state = msg.GearCmd
    
    def raw_command_callback(self, msg):
        
        self.throttle = msg.control.throttle
        self.brake = msg.control.brake
        self.steer = msg.control.steering_angle
        self.gear = msg.shift.data
        stamp = rospy.Time.now()
        
        # AEB 和 刹车功能设计
        # self.brake_msg.aeb_en_ctrl = rospy.get_param("aeb_en_ctrl", 0)
        self.brake_msg.header.stamp = stamp
        self.brake_msg.Brake_Pedal_Target = self.brake*100.0
        self.brake_msg.Brake_EnCtrl = 1

        self.steer_msg.header.stamp = stamp
        self.steer_msg.Steer_EnCtrl = 1
        self.steer_msg.Steer_AngleTarget = self.steer * steering_factor
        self.steer_msg.Steer_AngleSpeed = 250
        self.pub_steer.publish(self.steer_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.Gear_EnCtrl = 1
        if(self.gear==0):
            self.gear_msg.Gear_Target = 0
        elif(self.gear==1):
            self.gear_msg.Gear_Target = 1
        elif(self.gear==2):
            self.gear_msg.Gear_Target = 2
        elif(self.gear==3):
            self.gear_msg.Gear_Target = 3
        elif(self.gear==4):
            self.gear_msg.Gear_Target = 4
        
        self.pub_gear.publish(self.gear_msg)

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.Dirve_ThrottlePedalTarget = self.throttle*100.0
        self.throttle_msg.Dirve_EnCtrl = 1

        # 当档位不一致时，
        if(self.gear_state!=self.gear):
            self.throttle_msg.Dirve_ThrottlePedalTarget = 0
            self.brake_msg.Brake_Pedal_Target = 30
        
        self.pub_throttle.publish(self.throttle_msg)

        self.pub_brake.publish(self.brake_msg)

        self.vehicle_msg.header.stamp = stamp
        self.vehicle_msg.Drive_ModeCtrl = 0
        self.vehicle_msg.Steer_ModeCtrl = 1
        # self.vehicle_msg.vin_req = 1
        self.pub_vehicle.publish(self.vehicle_msg)



if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()