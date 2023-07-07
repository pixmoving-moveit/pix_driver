#!/usr/bin/env python

import rospy
# from pix_mini_ev_driver_msgs.msg import  gear_command_103, throttle_command_100, brake_command_101, vehicle_mode_command_105, steering_command_102 ,gear_report_503

from pix_mini_ev_driver_msgs.msg import eps_receive_314, sendx_364, msg100_100, msg101_101, msg104_104, msg204_204
#-------------------------------eps_receive_314/转向, sendx_364/制动, msg100_100/模式, msg101_101/油门, msg104_104/档位, msg204_204/档位反馈
from autoware_control_msgs.msg import ControlCommandStamped
from autoware_vehicle_msgs.msg import ShiftStamped
import math
import time

steering_factor = 500.0/(math.pi*(13.5/180))

class ControlConverter:
    def __init__(self):
        self.speed = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.gear = 0
        self.gear_act = 3
        self.driver_mode = 0
        self.pre_gear = 3
        
        self.sub_control_command = rospy.Subscriber('/control/control_cmd', ControlCommandStamped, self.control_command_callback)
        self.sub_shift_command = rospy.Subscriber('/control/shift_cmd', ShiftStamped, self.shift_callback)
        self.sub_gear_report = rospy.Subscriber('/pix/gear_report', msg204_204, self.gear_report_callback)

        self.pub_throttle = rospy.Publisher('/pix/throttle_command', msg101_101, queue_size=10)
        self.pub_brake = rospy.Publisher('/pix/brake_command', sendx_364, queue_size=10)
        self.pub_steer = rospy.Publisher('/pix/steering_command', eps_receive_314, queue_size=10)
        self.pub_gear = rospy.Publisher('/pix/gear_command', msg204_204, queue_size=10)
        self.pub_vehicle = rospy.Publisher('/pix/vehicle_mode_command', msg100_100, queue_size=10)

        self.throttle_msg = msg101_101()
        self.brake_msg = sendx_364()
        self.steer_msg = eps_receive_314()
        self.gear_msg = msg204_204()
        self.vehicle_msg = msg100_100()

    
    def control_command_callback(self, msg):
        self.speed = abs(msg.control.velocity)
        self.brake = msg.control.acceleration 
        self.steer = msg.control.steering_angle

        stamp = rospy.Time.now()

        if(self.gear != self.gear_act):
            self.speed = 0

        self.throttle_msg.header.stamp = stamp
        self.throttle_msg.Dirve_SpeedTarget = self.speed
        self.throttle_msg.Dirve_EnCtrl = 1
        
        self.pub_throttle.publish(self.throttle_msg)

        self.brake_msg.header.stamp = stamp
        self.brake_msg.Brake_Pedal_Target = self.brake*0
        self.brake_msg.Brake_EnCtrl = 1
        self.pub_brake.publish(self.brake_msg)

        self.steer_msg.header.stamp = stamp
        self.steer_msg.Steer_EnCtrl = 1
        self.steer_msg.Steer_AngleTarget = self.steer * steering_factor
        self.steer_msg.Steer_AngleSpeed = 250
        self.pub_steer.publish(self.steer_msg)

        self.gear_msg.header.stamp = stamp
        self.gear_msg.Gear_EnCtrl = 1
        self.gear_msg.Gear_Target = self.gear
        
        self.pub_gear.publish(self.gear_msg)

        self.driver_mode = 1
        self.vehicle_msg.header.stamp = stamp
        self.vehicle_msg.Drive_ModeCtrl = self.driver_mode
        self.vehicle_msg.Steer_ModeCtrl = 1
        self.pub_vehicle.publish(self.vehicle_msg)
    
    def shift_callback(self, msg):
        self.gear = msg.shift.data
    
    def gear_report_callback(self, msg):
        self.gear_act = msg.Gear_Actual
    




if __name__ == '__main__':
    
    rospy.init_node('autoware_pix_control_converter', anonymous=True)
    converter = ControlConverter()
    rospy.spin()