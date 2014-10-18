#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose, Point, PoseStamped, PointStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
import math

class My_pose():
	"""Simplified representation of relevant pose information"""
	def __init__(self, x=0.0, y=0.0, z=0.0):
		self.x = x #Linear
		self.y = y #Linear
		self.z = z #Rotational

class Robot():
	"""Representation of the robot. Governing its pose, goal, and behavior"""
	def __init__(self):
		self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.goal_pub = rospy.Publisher('/waypoint',PointStamped)
		self.pose_pub = rospy.Publisher('/robot_pose',PoseStamped)
		rospy.init_node('robot', anonymous=True)
		
		self.msg = Twist() #robot command that is published at 10Hz
		self.waypoints = [] #queue of waypoints
		self.pose = My_pose() #current pose

		self.goal = [0,0] #current goal

		self.cells_per_meter = 20
		self.path_sub = rospy.Subscriber('/dijkstra_path', Path, self.update_path, None)
		#self.odom = rospy.Subscriber('/odom', Odometry, self.update_odom, None)
		self.particle = rospy.Subscriber('/amcl_best',PoseStamped, self.update_particle, None)
		#self.clicked_point = rospy.Subscriber('/clicked_point',PointStamped, self.add_waypoint, None)

	def add_waypoint(self,msg):
		"""Obsolete function for manual adding waypoints from published rviz points"""
		self.waypoints.append([msg.point.x,msg.point.y])	

	def update_path(self,msg):
		"""Uses published Path to update waypoints queue"""
		waypoints = []
		for each in msg.poses:
			x = each.pose.position.x
			y = each.pose.position.y
			waypoints.append([x,y])
		self.waypoints = waypoints

	def update_odom(self,msg):
		"""Obsolete function for updating self.pose with odom info"""
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		z = math.atan2(2* (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w),1 - 2 * ((msg.pose.pose.orientation.y)**2 + (msg.pose.pose.orientation.z)**2))
		self.pose = My_pose(x,y,z)

	def update_particle(self,msg):
		"""Updates self.pose with pose from amcl update"""
		#Calculate simple Euler Yaw from Quaternion
		z = math.atan2(2* (msg.pose.orientation.z * msg.pose.orientation.w),1 - 2 * ((msg.pose.orientation.y)**2 + (msg.pose.orientation.z)**2))
		self.pose = My_pose(msg.pose.position.x,msg.pose.position.y,z)

	def to_waypoint(self):
		"""Calculates angle and distance between current position and the goal"""

		d = ((self.goal[0] - self.pose.x)**2 + (self.goal[1] - self.pose.y)**2)**(.5)
		z =  math.atan2((self.goal[1]-self.pose.y),(self.goal[0]-self.pose.x))

		return (d,z)


	def path_follow(self):
		"""Logic for updating msg with new command to move towards and find each waypoint"""

		if len(self.waypoints) > 0:
			self.goal = self.waypoints[0]
			(d,desired_z) = self.to_waypoint()
			
			#Threshold distance for when a waypoint if reached
			if (d < .2):
				self.waypoints.pop(0) #remove reached waypoint from queue
				self.msg = Twist() #stop if last waypoint reached
				return

			ang_vel = 0

			c = .2 #Angular velocity proportinal control constant

			dist = (desired_z - self.pose.z) #Angular deviation from goal

			#This block governs calculating the proper angular velocity
			if  abs(dist) < math.pi: #If we are less than 180 degrees off spin in this direction
				ang_vel = c * dist
			else: #The case where we are more than 180 degrees off spin the opposite direction
				if dist < 0:
					ang_vel = c * (2*math.pi - abs(dist)) #Ensure proportionality relative to actual angle deviation
				else:
					ang_vel = -1.0 * c * (2*math.pi - abs(dist))


			#print "Goal: "+str(self.goal) + " goal_ang: "+ str(desired_z) + " actual: " + str(self.pose.z) +" ang_vel: "+ str(ang_vel)+ "\n d: " + str(d) + " pos: " +str(self.pose.x)+", "+str(self.pose.y)

			lin_vel = 0

			#Forward movement only allowed if goal lies within the narrow pi/5 arch directly in front of the robot
			if abs(dist) < (math.pi/10) or (2*math.pi - abs(dist)) < (math.pi/10):
				lin_vel = min(.2, .1 * d) #max linear velocity of .2

			self.msg = Twist(linear = Vector3(x = lin_vel), angular = Vector3(z = ang_vel))
			
			#Dead reckoning "interpolation" updates

			s = 10 #Robot should be moving 1/10th of our velocities every 1/10th of a second
			#We found that increasing this term (decreasing estimate velocities) could yield more accurate estimations on carpet
			self.pose.x += (lin_vel/s) * math.cos(self.pose.z)
			self.pose.y += (lin_vel/s) * math.sin(self.pose.z)
			self.pose.z += (ang_vel/s)

		else:
			#When our waypoint queue is empty
			print("Done with Path")


	def run(self):
		"""Main function of running robot"""
		r=rospy.Rate(10)
		while not rospy.is_shutdown():
			self.path_follow() #Update robot Twist command
			self.cmd.publish(self.msg) #Publish Twist command

			ps = PointStamped(header=Header(stamp=rospy.Time.now(),frame_id="map"),point = Point(x=self.goal[0],y=self.goal[1]))
			self.goal_pub.publish(ps) #Publish current goal for rviz

			w = math.cos(self.pose.z/2)
			z = math.sin(self.pose.z/2)
			pose = PoseStamped(header=Header(stamp=rospy.Time.now(),frame_id="map"),pose = Pose(position = Point(x = self.pose.x, y = self.pose.y),orientation = Quaternion(w = w,z = z)))
			self.pose_pub.publish(pose) #Convert current pose to use Quaternion and publish for rviz
			
			r.sleep()

if __name__ == "__main__":
	r = Robot()
	r.run()