#!/usr/bin/env python3
import math

import rospy
from sensor_msgs.msg import JointState

from lio_sam.msg import M20JointsData


JOINT_NAMES = (
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
)


class M20JointStateAdapter:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/JOINTS_DATA")
        output_topic = rospy.get_param("~output_topic", "/joint_states_m20")
        self.drop_stale = bool(rospy.get_param("~drop_stale", True))
        self.position_scale = self._vector_param("~position_scale", 12, 1.0)
        self.position_offset = self._vector_param("~position_offset", 12, 0.0)
        self.wheel_velocity_scale = self._vector_param("~wheel_velocity_scale", 4, 1.0)
        self.wheel_position = [0.0] * 4
        self.last_source_stamp = None
        self.last_source_sequence = None

        self.publisher = rospy.Publisher(output_topic, JointState, queue_size=1000)
        self.subscriber = rospy.Subscriber(
            input_topic,
            M20JointsData,
            self.callback,
            queue_size=2000,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "[lio_sam][m20-adapter] %s -> %s (drop_stale=%s)",
            input_topic,
            output_topic,
            self.drop_stale,
        )

    @staticmethod
    def _vector_param(name, length, default):
        value = rospy.get_param(name, [default] * length)
        if not isinstance(value, list) or len(value) != length:
            raise rospy.ROSInitException("%s must contain %d values" % (name, length))
        result = [float(item) for item in value]
        if not all(math.isfinite(item) for item in result):
            raise rospy.ROSInitException("%s contains a non-finite value" % name)
        return result

    @staticmethod
    def _source_stamp(msg):
        if msg.source_stamp != rospy.Time():
            return msg.source_stamp
        return msg.header.stamp

    def callback(self, msg):
        if msg.stale and self.drop_stale:
            rospy.logwarn_throttle(2.0, "[lio_sam][m20-adapter] dropping stale JOINTS_DATA")
            return

        source_stamp = self._source_stamp(msg)
        positions = [
            msg.left_front_hip_x, msg.left_front_hip_y, msg.left_front_knee,
            msg.right_front_hip_x, msg.right_front_hip_y, msg.right_front_knee,
            msg.left_back_hip_x, msg.left_back_hip_y, msg.left_back_knee,
            msg.right_back_hip_x, msg.right_back_hip_y, msg.right_back_knee,
        ]
        wheel_velocity = [
            msg.left_front_wheel_speed,
            msg.right_front_wheel_speed,
            msg.left_back_wheel_speed,
            msg.right_back_wheel_speed,
        ]

        positions = [
            self.position_scale[i] * float(value) + self.position_offset[i]
            for i, value in enumerate(positions)
        ]
        wheel_velocity = [
            self.wheel_velocity_scale[i] * float(value)
            for i, value in enumerate(wheel_velocity)
        ]
        if not all(math.isfinite(value) for value in positions + wheel_velocity):
            rospy.logwarn_throttle(2.0, "[lio_sam][m20-adapter] dropping non-finite joint data")
            return

        source_changed = (
            self.last_source_sequence != msg.source_sequence
            or self.last_source_stamp != source_stamp
        )
        is_new_source = source_changed and not msg.repeated
        if is_new_source and self.last_source_stamp is not None:
            dt = (source_stamp - self.last_source_stamp).to_sec()
            if 0.0 < dt <= 0.5:
                for i in range(4):
                    self.wheel_position[i] += wheel_velocity[i] * dt

        if is_new_source:
            self.last_source_stamp = source_stamp
            self.last_source_sequence = msg.source_sequence

        output = JointState()
        output.header.stamp = source_stamp
        output.header.frame_id = "base_link"
        output.name = list(JOINT_NAMES)
        output.position = [
            positions[0], positions[1], positions[2], self.wheel_position[0],
            positions[3], positions[4], positions[5], self.wheel_position[1],
            positions[6], positions[7], positions[8], self.wheel_position[2],
            positions[9], positions[10], positions[11], self.wheel_position[3],
        ]
        output.velocity = [
            0.0, 0.0, 0.0, wheel_velocity[0],
            0.0, 0.0, 0.0, wheel_velocity[1],
            0.0, 0.0, 0.0, wheel_velocity[2],
            0.0, 0.0, 0.0, wheel_velocity[3],
        ]
        output.effort = [0.0] * len(JOINT_NAMES)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("m20_joint_state_adapter", anonymous=False)
    M20JointStateAdapter()
    rospy.spin()
