#!/usr/bin/env python

import rospy
# from pix_mini_ev_driver_msgs.msg import steering_report_502, gear_report_503, vcu_report_505 
from pix_mini_ev_driver_msgs.msg import eps_send_18f, msg204_204
from geometry_msgs.msg import TwistStamped
from autoware_vehicle_msgs.msg import Steering, ShiftStamped, TurnSignal, ControlMode
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math


steer_factor = (math.pi * (26.0/180)) / 240.0

class ReportConverter:
    def __init__(self):

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.shift = 0
        self.turn_signal = 0
        self.steer = 0.0
        self.mode = 0

        self.sub_steer = rospy.Subscriber("/pix/steering_report", eps_send_18f, self.steer_callback)
        self.sub_gear = rospy.Subscriber("/pix/gear_report", msg204_204, self.shift_callback)
        self.sub_vcu = rospy.Subscriber("/pix/vcu_report", msg204_204, self.vcu_callback)
        self.sub_imu = rospy.Subscriber("/sensing/camera/zed2i/zed_node/imu/data", Imu, self.imu_callback)

        self.pub_steer = rospy.Publisher("/vehicle/status/steering", Steering, queue_size=1)
        self.pub_shift = rospy.Publisher("/vehicle/status/shift", ShiftStamped, queue_size=1)
        self.pub_turn_signal = rospy.Publisher("/vehicle/status/turn_signal", TurnSignal, queue_size=1)
        self.pub_control_mode = rospy.Publisher("/vehicle/status/control_mode", ControlMode, queue_size=1)
        self.pub_twist = rospy.Publisher("/vehicle/status/twist", TwistStamped, queue_size=1)
        self.pub_velocity = rospy.Publisher("/vehicle/status/velocity", Float32, queue_size=1)

        self.twist_msg = TwistStamped()
        self.steer_msg = Steering()
        self.shift_msg = ShiftStamped()
        self.turn_signal_msg = TurnSignal()
        self.control_mode_msg = ControlMode()
        self.velocity_msg = Float32()

        self.wheel_tread = rospy.get_param("/vehicle_info/wheel_tread")
        self.wheel_base = rospy.get_param("/vehicle_info/wheel_base")

        self.prev_hd = Header()
        self.imu_0 = Imu()
        self.seq = 0
    
    def steer_callback(self, msg):
        self.steer = -msg.Steer_AngleRear_Actual * steer_factor
        self.steer_msg.header.frame_id = "base_link"
        self.steer_msg.header.stamp = msg.header.stamp
        self.steer_msg.data = self.steer
        # publish autoware steer msg 
        self.pub_steer.publish(self.steer_msg)
    
    def vcu_callback(self, msg):
        self.linear_velocity = msg.Vehicle_Speed
        status = msg.Vehicle_ModeState
        if(status==0):
            self.mode = 0
        elif(status==1 or status==3):
            self.mode = 1
        self.turn_signal = msg.TurnLight_Actual

        self.velocity_msg.data = self.linear_velocity
        self.pub_velocity.publish(self.velocity_msg)

        self.control_mode_msg.header.stamp = msg.header.stamp
        self.control_mode_msg.data = self.mode
        self.pub_control_mode.publish(self.control_mode_msg)

        self.turn_signal_msg.header.frame_id = "base_link"
        self.turn_signal_msg.header.stamp = msg.header.stamp
        self.turn_signal_msg.data = self.turn_signal
        self.pub_turn_signal.publish(self.turn_signal_msg)
    
    def imu_callback(self, msg):
        self.angular_velocity = msg.angular_velocity.z
        if(self.seq == 0):
            self.linear_velocity = 0
            self.prev_hd = msg.header
            self.seq = self.seq + 1
            self.imu_0 = msg
        else:
            dt = msg.header.stamp.to_sec() - self.prev_hd.stamp.to_sec()
            self.prev_hd = msg.header
            linear_acc_x = msg.linear_acceleration.x - self.imu_0.linear_acceleration.x
            if(linear_acc_x > 0.01 or linear_acc_x < -0.01):
                linear_acc_x = 0
            self.linear_velocity = self.linear_velocity + linear_acc_x * dt
            self.seq = self.seq + 1
        self.twist_msg.header.frame_id = "base_link"
        self.twist_msg.header.stamp = msg.header.stamp
        self.twist_msg.twist.linear.x = self.linear_velocity
        self.twist_msg.twist.angular.z = self.angular_velocity
        self.pub_twist.publish(self.twist_msg)
    
    def shift_callback(self, msg):
        self.shift = msg.Gear_Actual
        
        self.shift_msg.header.frame_id = "base_link"
        self.shift_msg.header.stamp = msg.header.stamp
        self.shift_msg.shift.data = self.shift
        self.pub_shift.publish(self.shift_msg)

if __name__ == '__main__':
    rospy.init_node('autoware_pix_report_converter', anonymous=True)
    converter = ReportConverter()
    rospy.spin()

    



