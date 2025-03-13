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

        self.publish_vel = False  # A 버튼을 누르면 True, B 버튼을 누르면 False

        # 🔹 50ms (0.05초)마다 실행되는 타이머 생성
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("JoystickTeleop 노드가 시작되었습니다.")

        # 🔹 게임패드 입력을 별도의 스레드에서 처리
        self.create_timer(0.01, self.joy_callback)  # 10ms마다 입력 체크

    def joy_callback(self):
        """ 게임패드 입력을 읽어 버튼 상태를 업데이트 """
        pygame.event.pump()

        btn_A = self.joystick.get_button(0)  # 이동 시작
        btn_B = self.joystick.get_button(1)  # 정지

        if btn_A:
            self.publish_vel = True  # A 버튼을 누르면 속도 퍼블리시 시작

        if btn_B:
            self.publish_vel = False  # B 버튼을 누르면 즉시 정지
            self.send_zero_velocity()

    def timer_callback(self):
        """ 주기적으로 Twist 메시지를 퍼블리시하는 함수 """
        twist = Twist()

        if self.publish_vel:
            # 🔹 일반적인 조이스틱 입력 값 반영
            linear_x = math.trunc(-self.joystick.get_axis(1) * 10) / 10
            angular_z = math.trunc(self.joystick.get_axis(2) * 10) / 10

            twist.linear.x = linear_x * self.linear_scale
            twist.angular.z = -angular_z * self.angular_scale
            self.cmd_vel_pub.publish(twist)

    def send_zero_velocity(self):
        """ 정지 명령을 즉시 퍼블리시하는 함수 """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()

    # 🔹 멀티스레드 실행: pygame 입력 처리 & ROS 2 타이머 콜백 병렬 실행
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
