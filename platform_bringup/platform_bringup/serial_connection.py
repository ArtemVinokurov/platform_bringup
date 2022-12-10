#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from platform_bringup.serial_server import SerialServer
from platform_msg.msg import CmdLinear, FeedbackData
from math import *

class PlatformSerialConnection(Node):
    def __init__(self):
        super().__init__('platform_serial_connection')
        port = '/dev/ttyACM0'
        self.serial = SerialServer(port)
        if self.serial is None:
            self.destroy_node()
        self.sub = self.create_subscription(CmdLinear, '/cmd_linear', self.cmd_linear_callback, 10)

        self.pub_feedback = self.create_publisher(FeedbackData, '/platform_feedback', 10)
        self.feedback_timer = self.create_timer(0.01, self.serial_feedback)

        self.cmd = CmdLinear()
        self.cmd.vel = 0.0
        self.cmd.step = 0.0
        self.steps_on_rev = 800  # число шагов на оборот шагового двигателя

        self.d = 0.056 # диаметр шестерни, м
        self.ratio = 24 # передаточное число редуктора

     
    def cmd_linear_callback(self, msg=CmdLinear()):
        if msg.vel != self.cmd.vel or msg.step != self.cmd.step:
            self.cmd = msg
            self.send_cmd_linear()


    # отправка команды на контроллер в виде "p 10000 10000"
    def send_cmd_linear(self):
        step_vel, step_pos = self.convert_to_steps(self.cmd.vel, self.cmd.step)
        cmd_str = "p" + " " + str(step_vel) + " " + str(step_pos)
        print(cmd_str)
        self.serial.send_cmd(cmd_str)

    
    def convert_to_steps(self, vel, step):
        
        s = self.cmd.step # длина шага платформы, м
        v = self.cmd.vel # скорость перешагивания платформы, м/с

        number_of_rev = s / (pi * self.d)  # суммарное число оборотов шестерни
        angular_vel = abs(60 * v / (pi * self.d)) # скорость вращения шестерни, об/мин

        step_pos = int(number_of_rev * self.steps_on_rev * self.ratio)
        step_vel = int(angular_vel * self.steps_on_rev * self.ratio / 60)

        if v < 0:
            step_pos *= -1
            
        return step_vel, step_pos

    # Перевод шагов двигателя в метры (для поступательного) и радианы (для вращательного)
    def convert_from_steps(self, p_steps, r_steps):
        p_joint_pos = (p_steps / self.steps_on_rev) * pi * self.d / self.ratio
        r_joint_pos =  r_steps / self.steps_on_rev * 2*pi / self.ratio
        return float(p_joint_pos), float(r_joint_pos)

    # Функция для парсинга входящих данных
    def parsing(self, pkg=str):
        begin_index = pkg.find('#')
        length = len(pkg)

        if begin_index >= 0 and begin_index + 1 < len(pkg) and pkg[begin_index+1] == ' ':
            temp_str = pkg[begin_index+1:]
            end_index = temp_str.find('#')
            result_str = temp_str[:end_index]
            data_list = result_str.split()
            return data_list
        else:
            return None


    # Обратная связь по сериалу в виде строки: "# 1 1 0 0 1000 1000 #" 

    def serial_feedback(self):
        data = self.serial.recieve_cmd()
        if data is not None:
            data_list = self.parsing(data)
            if data_list is not None:
                new_data_list = [int(item) for item in data_list]
                # print(new_data_list)

                feedback_msg = FeedbackData()

                feedback_msg.central_magnets = bool(new_data_list[0])
                feedback_msg.side_magnets = bool(new_data_list[1])
                feedback_msg.central_pneumo = bool(new_data_list[2])
                feedback_msg.side_pneumo = bool(new_data_list[3])
                feedback_msg.p_joint_pos, feedback_msg.r_joint_pos = self.convert_from_steps(new_data_list[4], new_data_list[5])

                self.pub_feedback.publish(feedback_msg)
        else:
            print('No data')


def main():
    rclpy.init()
    node = PlatformSerialConnection()
    while rclpy.ok():
        rclpy.spin_once(node)


if __name__ == '__main__':
    main()