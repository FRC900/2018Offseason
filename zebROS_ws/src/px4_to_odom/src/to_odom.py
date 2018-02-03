#! /usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from px_comm.msg import OpticalFlow

global prev_time
pose_position_x, pose_position_y = 0, 0

pub = rospy.Publisher("px4_odom", Odometry, queue_size=1)

def listener ():
	rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, callback)

def callback (data):
	global prev_time, pose_position_x, pose_position_y
	cur_time = rospy.get_rostime().to_sec()
	dt = cur_time - prev_time
	prev_time = cur_time
	odom = Odometry()
	odom.header.stamp = data.header.stamp
	odom.header.frame_id = "odom"
	odom.child_frame_id = "base_link"
	odom.pose.pose.orientation.w = 1
	odom.twist.twist.linear.x = data.velocity_x
	odom.twist.twist.linear.y = data.velocity_y

        # the first two variables here are necessary because the message attributes don't work with += for some reason
        pose_position_x += data.velocity_x / dt
        pose_position_y += data.velocity_y / dt
	odom.pose.pose.position.x = pose_position_x
	odom.pose.pose.position.y = pose_position_y

        pub.publish(odom)


if __name__ == "__main__":
	global prev_time
	rospy.init_node("to_odom")
	prev_time = 0
	listener()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()
		rospy.spin()
