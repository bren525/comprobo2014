#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap
import heapq

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
from std_msgs.msg import Header


class Cell:
	"""Simple representation of each cell as it flows through the Dijkstra algorithm"""
	def __init__(self,x,y):
		self.x = x
		self.y = y
		self.visited = False
		self.cost = 1230000000.0 #roughly inifinite, characteristic number for debug


class Map:
	def __init__(self):
		"""Fetches map and defines map constants"""

		rospy.wait_for_service("static_map")
		static_map = rospy.ServiceProxy("static_map", GetMap)
		try:
			self.map = static_map().map
		except:
			print "error receiving map"

		self.xrange = [0,512] #Zooms in on section of given map
		self.yrange = [0,512] #Unecessary in final implementation

	def getOccupied(self,center_x,center_y):
		"""Returns positive integer if this map cell is occupied or within the occupation buffer. 
			Returns 0 if this cell is unoccupied. Returns -1 if this cell's occupancy is unknown""" 

		if self.map.data[(center_y+self.yrange[0])+(center_x+self.xrange[0])*self.map.info.width] > 0:
			return 2
		#Array represents all cells in buffer around center point, currently a 15x15 square
		for c in [(-7, -7), (-7, -6), (-7, -5), (-7, -4), (-7, -3), (-7, -2), (-7, -1), (-7, 0), (-7, 1), (-7, 2), (-7, 3), (-7, 4), (-7, 5), (-7, 6), (-7, 7), (-6, -7), (-6, -6), (-6, -5), (-6, -4), (-6, -3), (-6, -2), (-6, -1), (-6, 0), (-6, 1), (-6, 2), (-6, 3), (-6, 4), (-6, 5), (-6, 6), (-6, 7), (-5, -7), (-5, -6), (-5, -5), (-5, -4), (-5, -3), (-5, -2), (-5, -1), (-5, 0), (-5, 1), (-5, 2), (-5, 3), (-5, 4), (-5, 5), (-5, 6), (-5, 7), (-4, -7), (-4, -6), (-4, -5), (-4, -4), (-4, -3), (-4, -2), (-4, -1), (-4, 0), (-4, 1), (-4, 2), (-4, 3), (-4, 4), (-4, 5), (-4, 6), (-4, 7), (-3, -7), (-3, -6), (-3, -5), (-3, -4), (-3, -3), (-3, -2), (-3, -1), (-3, 0), (-3, 1), (-3, 2), (-3, 3), (-3, 4), (-3, 5), (-3, 6), (-3, 7), (-2, -7), (-2, -6), (-2, -5), (-2, -4), (-2, -3), (-2, -2), (-2, -1), (-2, 0), (-2, 1), (-2, 2), (-2, 3), (-2, 4), (-2, 5), (-2, 6), (-2, 7), (-1, -7), (-1, -6), (-1, -5), (-1, -4), (-1, -3), (-1, -2), (-1, -1), (-1, 0), (-1, 1), (-1, 2), (-1, 3), (-1, 4), (-1, 5), (-1, 6), (-1, 7), (0, -7), (0, -6), (0, -5), (0, -4), (0, -3), (0, -2), (0, -1), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (1, -7), (1, -6), (1, -5), (1, -4), (1, -3), (1, -2), (1, -1), (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (1, 7), (2, -7), (2, -6), (2, -5), (2, -4), (2, -3), (2, -2), (2, -1), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (3, -7), (3, -6), (3, -5), (3, -4), (3, -3), (3, -2), (3, -1), (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), (3, 6), (3, 7), (4, -7), (4, -6), (4, -5), (4, -4), (4, -3), (4, -2), (4, -1), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6), (4, 7), (5, -7), (5, -6), (5, -5), (5, -4), (5, -3), (5, -2), (5, -1), (5, 0), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5), (5, 6), (5, 7), (6, -7), (6, -6), (6, -5), (6, -4), (6, -3), (6, -2), (6, -1), (6, 0), (6, 1), (6, 2), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7), (7, -7), (7, -6), (7, -5), (7, -4), (7, -3), (7, -2), (7, -1), (7, 0), (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (7, 6), (7, 7)]:
			x = center_x + c[0]
			y = center_y + c[1]
			if (x > 0 and x < (self.xrange[1] - self.xrange[0])
				and y > 0 and y < (self.yrange[1] - self.yrange[0])
				and self.map.data[(y+self.yrange[0])+(x+self.xrange[0])*self.map.info.width] > 0):
				#print "Obstacle found "+str(y)+ ", " +str(x)+ " | " +str(y+self.yrange[0])+ ", " +str(x+self.xrange[0]) 
				return 1
		if self.map.data[(center_y+self.yrange[0])+(center_x+self.xrange[0])*self.map.info.width] < 0:
			return -1
		else:
			return 0

	def dijkstra(self, a, b):
		"""Builds grid of cells and uses map to run Dijkstra algorithm to plan path between 
			point a and point b"""
		grid = [] #grid of cells to keep track of Dijkstra info
		pq = [] #priority queue to search through cells

		#Build grid
		for x in range(self.xrange[1]-self.xrange[0]):
			grid.append([])
			for y in range(self.yrange[1]-self.yrange[0]):
				cell = Cell(x,y)
				grid[x].append(cell)
		
		#Add cell at point a to queue
		cell = grid[a[0]][a[1]]
		cell.cost = 0.0
		cell.previous = None
		heapq.heappush(pq,(cell.cost,cell))

		def adjacentCells(cell,grid):
			"""Returns a list of all adjacent cells of a given cell in a given grid"""
			cells = []
			for pair in [[-1,-1],[-1,0],[-1,1],[0,-1],[0,1],[1,-1],[1,0],[1,1]]:
				if(cell.x + pair[0] < len(grid) and cell.y + pair[1] < len (grid[0])
					and cell.x + pair[0] >= 0 and cell.y + pair[1] >= 0
					and grid[cell.x+pair[0]][cell.y + pair[1]].visited == False):
					cells.append(grid[cell.x+pair[0]][cell.y + pair[1]])
			return cells


		def costFunction(start,end,goal):
			"""Calculates the cost of moving from a start cell to an end cell.
				Uses goal cell for A* heuristic."""
			def dist(a,b,c,d):
				"""Calculates distance between (a,b) and (c,d)"""
				return (float(a-c)**2 + float(b-d)**2)**(.5)

			if(self.getOccupied(end.x,end.y) >= 1):
				return 321000000.0 #Effectively infinite cost of an occupied cell
			else:
				#Key cost formula for A* Dijkstra
				return dist(start.x,start.y,end.x,end.y) + dist(end.x,end.y,goal.x,goal.y)

		total = 0 #Total cell visit count used for efficiency measurements

		while(len(pq) > 0):
			#heapq built in python module turns our list into a min priority queue
			cell = heapq.heappop(pq)[1]
			cell.visited = True

			total += 1

			if(cell.x == b[0] and cell.y == b[1]):
				#When we've found our goal we are done
				break

			#Core dijkstra, looks at adjacent cells and updates cost
			for adjacent in adjacentCells(cell,grid):
				newCost = cell.cost + costFunction(cell,adjacent,grid[b[0]][b[1]])
				if(newCost < adjacent.cost):
					adjacent.cost = newCost
					adjacent.previous = cell
					heapq.heappush(pq,(adjacent.cost,adjacent))
				

		end = grid[b[0]][b[1]]
		print end.c #The final path's cost
		
		#Steps backwards from the goal's "previous" to reconstruct path from start 
		path = [[end.x,end.y]]
		while(end.previous != None):
			end = end.previous
			path.insert(0,[end.x,end.y])
			
		print path #The final path
		return path

class Path_Finder:
	def __init__(self):
		self.pub = rospy.Publisher('/dijkstra_path', Path, queue_size=10)
		rospy.init_node('path_planner', anonymous=True)
		self.start = [0,0]
		self.resolution = 20 #dots per meter

		self.clicked_point = rospy.Subscriber('/clicked_point',PointStamped, self.find_path, None)
		self.robot_pose_sub = rospy.Subscriber('/amcl_best', PoseStamped, self.update_start, None)


	def find_waypoints(self, path):
		"""Converts final path to waypoints by detecting direction changes in path"""

		waypoints = []
		cur_dir = (0,0)
		#Append each point where the direction changes
		for i in range(1,len(path)):
			new_dir = (path[i][0]-path[i-1][0],path[i][1]-path[i-1][1])
			if new_dir != cur_dir:
				waypoints.append(path[i-1])
				cur_dir = new_dir
		
		waypoints.append(path[len(path)-1]) #Append final goal
		
		return waypoints
					

	def publish_path(self, path):
		"""Convert final waypoint list to a publishable list of meter based coordinates
		and then published this Path for the path_follower."""
		poses = [] #List of path poses
		for each in path:
			#Must stamp poses with Header for rviz
			#Convert from our cell grid interger system to a float based meter coordinate system 
			#by dividing by the map resolution, 20 cells per meter.
			poses.append(PoseStamped(header = Header(stamp=rospy.Time.now(),frame_id="map"), pose = Pose(position = Point(y=float(each[0])/self.resolution,x=float(each[1])/self.resolution))))
		
		#Publish Path
		self.pub.publish(Path(header = Header(stamp=rospy.Time.now(),frame_id="map"), poses = poses))

	def plot_map(self, m,path):
		"""Utilizes matplotlib to plot our map and our path on the map"""

		plt.hold(True)
		plt.axis([0,m.xrange[1]-m.xrange[0],0,m.yrange[1]-m.yrange[0]])
		for x in range(m.xrange[1]-m.xrange[0]):
			for y in range(m.yrange[1]-m.yrange[0]):
				if m.getOccupied(x,y) == 2:
					plt.plot(x,y,'b.')
				if m.getOccupied(x,y) == 1:
					plt.plot(x,y,'y.')
		
		for point in path:
			plt.plot(point[0],point[1],'r.')
		plt.show()

	def update_start(self,msg):
		"""Each time amcl publishes a new best pose the Dijkstra starting position is updated"""
		#Convert from meters to cells by multiplying by map resolution 20 cells per meter
		self.start = [int(round(msg.pose.position.y*self.resolution)),int(round(msg.pose.position.x*self.resolution))]

	def find_path(self,msg):
		"""Each time a new point is selected in rviz, calculate and publish the path to that point"""

		#Convert goal from meters to cells
		goal = [int(round(msg.point.y*self.resolution)),int(round(msg.point.x*self.resolution))]
		print "Goal: " + str(goal)

		m = Map() #Build map
		path = m.dijkstra(self.start,goal) #Run dijkstra
		path = self.find_waypoints(path) #Convert to waypoints
		path.pop(0) #Removed starting point with assumption that robot is already there
		self.publish_path(path)
		#self.plot_map(m,path)

	def run(self):
		"""Keep node alive"""
		r=rospy.Rate(10)
		while not rospy.is_shutdown():
			r.sleep()


if __name__ == "__main__":
	pf = Path_Finder()
	pf.run()
	