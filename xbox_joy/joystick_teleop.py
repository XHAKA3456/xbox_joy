#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import time
import math
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop_node')
        pygame.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            self.get_logger().error("연결된 게임패드가 없습니다.")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.linear_axis_index = 1
        self.angular_axis_index = 2
        self.linear_scale = 0.2
        self.angular_scale = 0.4

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)

        self.publish_vel = False

        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("JoystickTeleop 노드가 시작되었습니다.")

        self.create_timer(0.01, self.joy_callback)

    def joy_callback(self):
        pygame.event.pump()

        btn_A = self.joystick.get_button(0)
        btn_B = self.joystick.get_button(1)

        if btn_A:
            self.publish_vel = True

        if btn_B:
            self.publish_vel = False
            self.send_zero_velocity()

    def timer_callback(self):
        twist = Twist()

        if self.publish_vel:
            linear_x = math.trunc(-self.joystick.get_axis(1) * 10) / 10
            angular_z = math.trunc(self.joystick.get_axis(2) * 10) / 10

            twist.linear.x = linear_x * self.linear_scale
            twist.angular.z = -angular_z * self.angular_scale
            self.cmd_vel_pub.publish(twist)

    def send_zero_velocity(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
