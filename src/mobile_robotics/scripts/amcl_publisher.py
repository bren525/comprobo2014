#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header

covariance = [0.1,0.0,0.0,0.0,0.0,0.0,
			  0.0,0.1,0.0,0.0,0.0,0.0,
			  0.0,0.0,0.0,0.0,0.0,0.0,
			  0.0,0.0,0.0,0.0,0.0,0.0,
			  0.0,0.0,0.0,0.0,0.0,0.0,
			  0.0,0.0,0.0,0.0,0.0,0.1]
pose_set = False
def publish_pose(msg,pose_pub):
	pose = PoseStamped(header = Header(stamp=rospy.Time.now(),frame_id="map"), pose = msg.pose.pose)
	pose_pub.publish(pose)

def publish_initial(msg,initial_pub):
	global pose_set
	if(pose_set != True):
		pose = PoseWithCovarianceStamped(header = Header(stamp=rospy.Time.now(),frame_id="map"),pose = PoseWithCovariance(pose = Pose(position = msg.point,orientation = Quaternion(w = 1)), covariance = covariance))
		#initial_pub.publish(pose)
		print "Done setting pose..."
		pose_set = True

if __name__ == "__main__":
	rospy.init_node('amcl_pub', anonymous=True)


	pose_pub = rospy.Publisher('/amcl_best',PoseStamped)
	pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, publish_pose, pose_pub)

	initial_pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped)
	initial_sub = rospy.Subscriber('/clicked_point',PointStamped, publish_initial,initial_pub)
	
	r=rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()