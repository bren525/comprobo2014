#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from std_msgs.msg import Header


"""Simple node that translates the amcl's best estimated pose from a PoseWithCovarianceStamped
to a PoseStamped so that it can be displayed on rviz"""

def publish_pose(msg,pose_pub):
	pose = PoseStamped(header = Header(stamp=rospy.Time.now(),frame_id="map"), pose = msg.pose.pose)
	pose_pub.publish(pose)

if __name__ == "__main__":
	rospy.init_node('amcl_pub', anonymous=True)


	pose_pub = rospy.Publisher('/amcl_best',PoseStamped)
	pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, publish_pose, pose_pub)
	
	r=rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()