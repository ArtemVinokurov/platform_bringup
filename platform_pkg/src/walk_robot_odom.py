#!/usr/bin/env python3

from math import sin, cos, pi, atan2, asin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Quaternion, Vector3
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class WalkRobotTf(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('walk_robot_tf')

        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.timer_period = 0.001
        self.timer_broadcast = self.create_timer(self.timer_period, self.update_robot_pose)

        self.limit_prismatic_joint = [-0.14, 0.14]

        self.sub = self.create_subscription(
            JointState,
            'platform/joint_states',
            self.sensors_cb,
            10)

        self.state_sub = self.create_subscription(
            String,
            '/state_base',
            self.state_cb,
            10)

        self.joint_pos = [0.0, 0.0]
        self.prev_joint_pos = [0.0, 0.0]

        self.state_base = 'init'

        self.msg_joint_states = JointState()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 1.57

      

    def update_robot_pose(self):

        d_linear = self.joint_pos[0] - self.prev_joint_pos[0]
        d_angular = self.joint_pos[1] - self.prev_joint_pos[1]

       

        if d_linear != 0.0:
            
            if self.state_base == 'move':
                x = cos(d_angular) * d_linear
                y = sin(d_angular) * d_linear

                self.x = self.x - (cos(self.yaw) * x + sin(self.yaw) * y)
                self.y = self.y - (sin(self.yaw) * x - cos(self.yaw) * y)

        if d_angular != 0.0:
            self.yaw = self.yaw + d_angular

        
        self.prev_joint_pos[0] = self.joint_pos[0]
        self.prev_joint_pos[1] = self.joint_pos[1]            

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = sin(self.yaw / 2)
        q.w = cos(self.yaw / 2)

        now = self.get_clock().now()    
        self.odom_trans.header.stamp = now.to_msg()

        self.odom_trans.transform.translation.x = self.x
        self.odom_trans.transform.translation.y = self.y
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = q 

        self.broadcaster.sendTransform(self.odom_trans)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = d_linear / self.timer_period
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = d_angular / self.timer_period
        self.odom_pub.publish(odom)

        


    def sensors_cb(self, msg):
        if (msg.header.frame_id == 'base_link'):
            j1 = msg.position[0]
            j2 = -msg.position[1]
            self.joint_pos = [j1, j2]

    def state_cb(self, msg):
        self.state_base = msg.data
 
        
def main():
    node = WalkRobotTf()
    rclpy.spin(node)  

if __name__ == '__main__':
    main()