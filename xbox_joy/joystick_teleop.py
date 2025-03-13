#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop_node')
        pygame.init()
    
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("연결된 게임패드가 없습니다.")
            return
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.linear_axis_index = 1 
        self.angular_axis_index = 0 
        self.linear_scale = 0.2
        self.angular_scale = 0.3

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.publish_vel = False
        self.get_logger().info("JoystickTeleop 노드가 시작되었습니다.")
        self.joy_callback()

    def joy_callback(self):
        
        while True:

            twist = Twist()
            pygame.event.pump()
            btn_A = self.joystick.get_button(0)
            btn_B = self.joystick.get_button(1)
            if btn_A : 
                self.publish_vel = True
            if btn_B:
                self.publish_vel =False
                twist.linear.x = 0.0
                twist.angular.z =0.0
                self.cmd_vel_pub.publish(twist)

            
            linear_x = math.trunc(-self.joystick.get_axis(1) * 10) / 10
            angular_z = math.trunc(self.joystick.get_axis(2) * 10) / 10
            # print ("linear_X : ",linear_x, "  ||||||||||||  ","angular_z : ", angular_z)
            # print("btn _A :"  , btn_A , "  btn _B : " , btn_B)
            if self.publish_vel:
                twist.linear.x = linear_x*self.linear_scale
                twist.angular.z = -angular_z*self.angular_scale
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

