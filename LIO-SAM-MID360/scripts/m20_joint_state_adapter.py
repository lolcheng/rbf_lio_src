#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET

import rospy
from sensor_msgs.msg import JointState

from lio_sam.msg import M20JointsData


JOINT_NAMES = (
    "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
    "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
    "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
    "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
)

LEG_JOINT_NAMES = tuple(
    joint_name for index, joint_name in enumerate(JOINT_NAMES) if index % 4 != 3
)


def parse_urdf_position_limits(robot_description):
    try:
        root = ET.fromstring(robot_description)
    except ET.ParseError as exc:
        raise ValueError("robot_description is not valid XML: %s" % exc)

    urdf_joints = {joint.get("name"): joint for joint in root.findall("joint")}
    limits = []
    for joint_name in LEG_JOINT_NAMES:
        joint = urdf_joints.get(joint_name)
        limit = joint.find("limit") if joint is not None else None
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError("missing URDF position limits for %s" % joint_name)
        lower = float(limit.get("lower"))
        upper = float(limit.get("upper"))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
            raise ValueError("invalid URDF position limits for %s" % joint_name)
        limits.append((lower, upper))
    return tuple(limits)


def clamp_positions_to_limits(positions, limits):
    if len(positions) != len(limits):
        raise ValueError("position and limit lengths differ")
    return [max(lower, min(upper, value)) for value, (lower, upper) in zip(positions, limits)]


def convert_leg_positions(positions, scales, offsets, wrap_positions=False):
    if not len(positions) == len(scales) == len(offsets) == len(LEG_JOINT_NAMES):
        raise ValueError("M20 leg conversion requires 12 positions, scales and offsets")
    converted = [
        scale * float(position) + offset
        for position, scale, offset in zip(positions, scales, offsets)
    ]
    if not all(math.isfinite(value) for value in converted):
        raise ValueError("non-finite M20 leg position")
    # Encoder and model zeros may differ by a full turn. Select the equivalent
    # model angle before applying the URDF's bounded revolute-joint limits.
    if wrap_positions:
        converted = [math.atan2(math.sin(value), math.cos(value)) for value in converted]
    return converted


class M20JointStateAdapter:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/JOINTS_DATA")
        output_topic = rospy.get_param("~output_topic", "/joint_states_m20")
        self.drop_stale = bool(rospy.get_param("~drop_stale", True))
        self.position_scale = self._vector_param("~position_scale", 12, 1.0)
        self.position_offset = self._vector_param("~position_offset", 12, 0.0)
        self.wrap_positions = bool(rospy.get_param("~wrap_positions", False))
        self.wheel_velocity_scale = self._vector_param("~wheel_velocity_scale", 4, 1.0)
        try:
            self.position_limits = parse_urdf_position_limits(
                rospy.get_param("/robot_description")
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise rospy.ROSInitException(
                "cannot load M20 limits from /robot_description: %s" % exc
            )
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

        try:
            positions = convert_leg_positions(
                positions, self.position_scale, self.position_offset, self.wrap_positions
            )
        except (TypeError, ValueError, OverflowError):
            rospy.logwarn_throttle(2.0, "[lio_sam][m20-adapter] dropping invalid leg data")
            return
        wheel_velocity = [
            self.wheel_velocity_scale[i] * float(value)
            for i, value in enumerate(wheel_velocity)
        ]
        if not all(math.isfinite(value) for value in positions + wheel_velocity):
            rospy.logwarn_throttle(2.0, "[lio_sam][m20-adapter] dropping non-finite joint data")
            return

        clamped_positions = clamp_positions_to_limits(positions, self.position_limits)
        for joint_name, measured, clamped, limits in zip(
            LEG_JOINT_NAMES, positions, clamped_positions, self.position_limits
        ):
            if measured != clamped:
                rospy.logwarn_throttle(
                    2.0,
                    "[lio_sam][m20-adapter] %s=%.6f is outside URDF limits "
                    "[%.6f, %.6f]; clamped to %.6f",
                    joint_name,
                    measured,
                    limits[0],
                    limits[1],
                    clamped,
                )
        positions = clamped_positions

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
