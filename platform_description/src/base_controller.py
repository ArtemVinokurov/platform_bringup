#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import JointState


class BaseController(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('base_controller')
        self.sub_linear = self.create_subscription(Twist, '/cmd_linear', self.cmd_cb, 10)
        # self.pub_state = self.create_publisher(StateWalk, '/state_walk', 1)
        self.joint_state_pub = self.create_publisher(JointState, '/platform/joint_states', 10)
        self.walk_state_pub = self.create_publisher(String, '/state_base', 10)
        
        self.timer_period = 0.001
        self.timer_joint_pub = self.create_timer(self.timer_period, self.joint_pub_cb)
        self.timer_check_limit = self.create_timer(self.timer_period, self.check_limit_cb)
        
        self.joint_states_msg = JointState()

        self.cmd_linear_vel = 0
        self.cmd_angular_vel = 0

        self.limit_prismatic_joint = [-0.14, 0.14]
        self.joint_magnet_limit = [0, 0.05]

        self.upper_limit = False
        self.state_msg = String()

        self.state_msg.data = 'init'

        self.joint_states_msg.name.append("joint_1")
        self.joint_states_msg.name.append("joint_2")
        self.joint_states_msg.name.append("joint1_1")
        self.joint_states_msg.name.append("joint1_2")
        self.joint_states_msg.name.append("joint1_3")
        self.joint_states_msg.name.append("joint1_4")
        self.joint_states_msg.name.append("joint3_1")
        self.joint_states_msg.name.append("joint3_2")
        self.joint_states_msg.name.append("joint3_3")
        self.joint_states_msg.name.append("joint3_4")

        self.joint_states_msg.position.append(0)
        self.joint_states_msg.position.append(0)
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])
        self.joint_states_msg.position.append(self.joint_magnet_limit[1])


    def cmd_cb(self, msg):
        vel = msg.linear.y
        angular = msg.angular.z
        
        if vel != 0 and angular != 0:
            self.get_logger().info('Cmd velocity error: linear and angular velocity are not equal to zero at the same time')
        else:
            self.cmd_linear_vel = vel
            self.cmd_angular_vel = angular

    def check_limit_cb(self):

        if self.cmd_linear_vel < 0:
            if self.joint_states_msg.position[0] < self.limit_prismatic_joint[0]:
                self.upper_limit = True
            if self.joint_states_msg.position[0] > self.limit_prismatic_joint[1]:
                self.upper_limit = False

        else:
            if self.joint_states_msg.position[0] > self.limit_prismatic_joint[1]:
                self.upper_limit = True
            if self.joint_states_msg.position[0] < self.limit_prismatic_joint[0]:
                self.upper_limit = False

        

    def joint_pub_cb(self):

        if self.cmd_linear_vel != 0:
            if self.upper_limit:
                self.state_msg.data = 'move'
                self.joint_states_msg.position[0] -= self.cmd_linear_vel * self.timer_period
            else:
                self.state_msg.data = 'stay'  
                self.joint_states_msg.position[0] += self.cmd_linear_vel * self.timer_period

            self.up_down_magnit(self.upper_limit)

        if self.cmd_angular_vel != 0:
            self.state_msg.data = 'rotate'
            self.up_down_magnit(False)
            self.joint_states_msg.position[1] += (-1)*self.cmd_angular_vel * self.timer_period
        
        self.joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_msg.header.frame_id = 'base_link'
        self.joint_state_pub.publish(self.joint_states_msg)
        self.walk_state_pub.publish(self.state_msg)


    def up_down_magnit(self, flag):

        self.joint_states_msg.position[2] = self.joint_magnet_limit[int(not flag)]
        self.joint_states_msg.position[3] = self.joint_magnet_limit[int(not flag)]
        self.joint_states_msg.position[4] = self.joint_magnet_limit[int(not flag)]
        self.joint_states_msg.position[5] = self.joint_magnet_limit[int(not flag)]
        
        self.joint_states_msg.position[6] = self.joint_magnet_limit[int(flag)]
        self.joint_states_msg.position[7] = self.joint_magnet_limit[int(flag)]
        self.joint_states_msg.position[8] = self.joint_magnet_limit[int(flag)]
        self.joint_states_msg.position[9] = self.joint_magnet_limit[int(flag)]


def main():
    node = BaseController()
    rclpy.spin(node)


if __name__ == '__main__':
    main()


            