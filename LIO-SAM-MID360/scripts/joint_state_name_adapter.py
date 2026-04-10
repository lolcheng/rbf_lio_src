#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState


GENERIC_TO_TRON1A = {
    "joint_0": "abad_L_Joint",
    "joint_1": "hip_L_Joint",
    "joint_2": "knee_L_Joint",
    "joint_3": "wheel_L_Joint",
    "joint_4": "abad_R_Joint",
    "joint_5": "hip_R_Joint",
    "joint_6": "knee_R_Joint",
    "joint_7": "wheel_R_Joint",
}


class JointStateNameAdapter:
    def __init__(self):
        in_topic = rospy.get_param("~input_topic", "/joint_states")
        out_topic = rospy.get_param("~output_topic", "/joint_states_tron1a")
        self.refresh_stamp = bool(rospy.get_param("~refresh_stamp", True))

        self.pub = rospy.Publisher(out_topic, JointState, queue_size=200)
        self.sub = rospy.Subscriber(in_topic, JointState, self.callback, queue_size=200, tcp_nodelay=True)
        rospy.loginfo("[lio_sam][joint-adapter] %s -> %s (refresh_stamp=%s)", in_topic, out_topic, str(self.refresh_stamp))

    def callback(self, msg):
        if msg is None:
            return

        out = JointState()
        out.header = msg.header
        if self.refresh_stamp:
            out.header.stamp = rospy.Time.now()

        out.name = [GENERIC_TO_TRON1A.get(n, n) for n in msg.name]
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("joint_state_name_adapter", anonymous=False)
    JointStateNameAdapter()
    rospy.spin()
