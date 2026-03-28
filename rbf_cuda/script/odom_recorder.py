#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

output_file = "/root/cuda_rbf_ws/src/result/fast_lio_trajectory.tum"
f_out = open(output_file, "w")

def odom_callback(msg):
    timestamp = msg.header.stamp.to_sec()
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    f_out.write(f"{timestamp:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")

def listener():
    rospy.init_node('lio_sam_traj_recorder')
    # rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, odom_callback)
    rospy.Subscriber("/Odometry", Odometry, odom_callback)
    rospy.spin()
    f_out.close()

if __name__ == '__main__':
    listener()
