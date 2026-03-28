#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
from sensor_msgs.msg import JointState
from urdfpy import URDF
from geometry_msgs.msg import TransformStamped
import numpy as np

class ForwardKinematicsTFBroadcaster:
    def __init__(self):
        # 1. 加载 URDF 模型
        urdf_path = rospy.get_param('~urdf_path', '/root/code/robot-description/pointfoot/WF_TRON1A/urdf/robot.urdf')
        self.robot = URDF.load(urdf_path)

        # 2. 初始化 TF 广播器
        self.br = tf2_ros.TransformBroadcaster()

        # 3. 建立关节名称到关节对象的映射
        self.joint_map = {joint.name: joint for joint in self.robot.joints}

        # 4. 获取目标 link 名称
        self.target_links = ['wheel_R_Link', 'wheel_L_Link']

        # 5. 设置基准 link
        # self.base_link = 'base_link'
        self.base_link = self.robot.base_link.name
        print(f"URDF base link is: {self.robot.base_link.name}")

        # 6. 关节名称映射字典
        self.joint_name_map = {
            "joint_0": "abad_L_Joint",
            "joint_1": "hip_L_Joint",
            "joint_2": "knee_L_Joint",
            "joint_3": "wheel_L_Joint",
            "joint_4": "abad_R_Joint",
            "joint_5": "hip_R_Joint",
            "joint_6": "knee_R_Joint",
            "joint_7": "wheel_R_Joint"
        }

        # 7. 建立订阅
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        # 1. 构建关节位置字典，使用映射字典
        joint_positions = {}
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_name_map:
                mapped_joint_name = self.joint_name_map[joint_name]
                joint_positions[mapped_joint_name] = msg.position[i]

        # 2. 计算正运动学（forward kinematics）
        link_poses = self.robot.link_fk(cfg=joint_positions)

        # 3. 为每个目标 link 发布 TF
        for link_name in self.target_links:
            if link_name in self.robot.link_map:  # 确保 link 存在于 URDF 中
                link = self.robot.link_map[link_name]
                pose_matrix = link_poses[link]

                # 转换为平移和四元数
                translation = pose_matrix[:3, 3]
                rotation = tf_conversions.transformations.quaternion_from_matrix(pose_matrix)

                # 创建 TransformStamped 消息
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = self.base_link
                t.child_frame_id = link_name
                t.transform.translation.x = translation[0]
                t.transform.translation.y = translation[1]
                t.transform.translation.z = translation[2]
                t.transform.rotation.x = rotation[0]
                t.transform.rotation.y = rotation[1]
                t.transform.rotation.z = rotation[2]
                t.transform.rotation.w = rotation[3]

                # 发布 TF
                self.br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('forward_kinematics_tf_broadcaster')
    try:
        ForwardKinematicsTFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
