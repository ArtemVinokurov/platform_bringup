#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from platform_msg.msg import FeedbackData

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import JointState


class Publisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('publisher_joint_state')
        self.sub = self.create_subscription(FeedbackData, '/platform_feedback', self.pub_joint_state, 10)
        self.walk_state_pub = self.create_publisher(String, '/state_base', 10)

        self.pub = self.create_publisher(JointState, '/platform/joint_states', 10)
    
        
        self.joint_states_msg = JointState()

        self.limit_prismatic_joint = [-0.14, 0.14]
        self.pneumo_joint_limit = [0, 0.05]

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
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])
        self.joint_states_msg.position.append(self.pneumo_joint_limit[1])

    def pub_joint_state(self, msg=FeedbackData):

        feedback = msg
        if feedback.side_pneumo and not feedback.central_pneumo:
            self.state_msg.data = 'move_base'
        else:
            self.state_msg.data = 'stay'        


        self.joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_msg.header.frame_id = 'base_link'
        self.joint_states_msg.position[0] = feedback.p_joint_pos
        self.joint_states_msg.position[1] = feedback.r_joint_pos
        self.up_down_side_pneumo(feedback.side_pneumo)
        self.up_down_central_pneumo(feedback.central_pneumo)

        self.walk_state_pub.publish(self.state_msg)
        self.pub.publish(self.joint_states_msg)


    def up_down_side_pneumo(self, flag):

        self.joint_states_msg.position[2] = self.pneumo_joint_limit[int(flag)]
        self.joint_states_msg.position[3] = self.pneumo_joint_limit[int(flag)]
        self.joint_states_msg.position[4] = self.pneumo_joint_limit[int(flag)]
        self.joint_states_msg.position[5] = self.pneumo_joint_limit[int(flag)]
        

    def up_down_central_pneumo(self, flag):    
        self.joint_states_msg.position[6] = self.pneumo_joint_limit[int(flag)]
        self.joint_states_msg.position[7] = self.pneumo_joint_limit[int(flag)]
        self.joint_states_msg.position[8] = self.pneumo_joint_limit[int(flag)]
        self.joint_states_msg.position[9] = self.pneumo_joint_limit[int(flag)]


def main():
    node = Publisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
