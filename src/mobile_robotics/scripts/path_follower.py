#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point, PoseStamped, PointStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
import math

class My_pose():
	def __init__(self, x=0.0, y=0.0, z=0.0):
		self.x = x
		self.y = y
		self.z = z

class Robot():
	def __init__(self):
		self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.goal_pub = rospy.Publisher('/waypoint',PointStamped)
		self.pose_pub = rospy.Publisher('/robot_pose',PoseStamped)
		rospy.init_node('robot', anonymous=True)
		self.msg = Twist()
		self.waypoints = []
		#self.waypoints = [[-1.5,-1.5],[-1.7,.2],[.5,.5]]
		self.pose = My_pose()
		self.origin = [0,0]
		self.goal = [0,0]

		self.cells_per_meter = 20
		self.path_sub = rospy.Subscriber('/dijkstra_path', Path, self.update_path, None)
		#self.odom = rospy.Subscriber('/odom', Odometry, self.update_odom, None)
		self.particle = rospy.Subscriber('/amcl_best',PoseStamped, self.update_particle, None)
		#self.clicked_point = rospy.Subscriber('/clicked_point',PointStamped, self.add_waypoint, None)

	def add_waypoint(self,msg):
		self.waypoints.append([msg.point.x,msg.point.y])	

	def update_path(self,msg):
		waypoints = []
		for each in msg.poses:
			x = each.pose.position.x
			y = each.pose.position.y
			waypoints.append([x,y])
		self.waypoints = waypoints

	def update_odom(self,msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		z = math.atan2(2* (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w),1 - 2 * ((msg.pose.pose.orientation.y)**2 + (msg.pose.pose.orientation.z)**2))
		self.pose = My_pose(x,y,z)

	def update_particle(self,msg):
		z = math.atan2(2* (msg.pose.orientation.z * msg.pose.orientation.w),1 - 2 * ((msg.pose.orientation.y)**2 + (msg.pose.orientation.z)**2))
		self.pose = My_pose(msg.pose.position.x,msg.pose.position.y,z)

	def to_waypoint(self,x,y):
		d = ((x - self.pose.x)**2 + (y - self.pose.y)**2)**(.5)
		z =  math.atan2((y-self.pose.y),(x-self.pose.x))
		return (d,z)


	def path_follow(self):
		if len(self.waypoints) > 0:
			self.goal = self.waypoints[0]
			(d,desired_z) = self.to_waypoint(self.goal[0],self.goal[1])
			if (d < .2):
				self.waypoints.pop(0)
				self.msg = Twist()
				return

			ang_vel = 0
			c = .2
			dist = (desired_z - self.pose.z)

			if  abs(dist) < math.pi:
				ang_vel = c * dist
			else:
				if dist < 0:
					ang_vel = c * (2*math.pi - abs(dist))
				else:
					ang_vel = -1.0 * c * (2*math.pi - abs(dist))


			print "Goal: "+str(self.goal) + " goal_ang: "+ str(desired_z) + " actual: " + str(self.pose.z) +" ang_vel: "+ str(ang_vel)+ "\n d: " + str(d) + " pos: " +str(self.pose.x)+", "+str(self.pose.y)

			lin_vel = 0
			if math.fabs((desired_z - self.pose.z)) < (math.pi/10) or (2*math.pi - abs(dist)) < (math.pi/10):
				lin_vel = min(.2, .1 * d)

			self.msg = Twist(linear = Vector3(x = lin_vel), angular = Vector3(z = ang_vel))
			#Jank Odom
			s = 12
			self.pose.x += (lin_vel/s) * math.cos(self.pose.z)
			self.pose.y += (lin_vel/s) * math.sin(self.pose.z)
			self.pose.z += (ang_vel/s)

		else:
			print("Done with Path")


	def run(self):
		r=rospy.Rate(10)
		while not rospy.is_shutdown():
			self.path_follow()
			self.cmd.publish(self.msg)
			ps = PointStamped(header=Header(stamp=rospy.Time.now(),frame_id="map"),point = Point(x=self.goal[0],y=self.goal[1]))
			self.goal_pub.publish(ps)
			w = math.cos(self.pose.z/2)
			z = math.sin(self.pose.z/2)
			pose = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id="map"),pose = Pose(position = Point(x = self.pose.x, y = self.pose.y),orientation = Quaternion(w = w,z = z)))
			self.pose_pub.publish(pose)
			r.sleep()

if __name__ == "__main__":
	r = Robot()
	r.run()