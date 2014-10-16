#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap
import heapq

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point


class Cell:
	def __init__(self,x,y):
		self.x = x
		self.y = y
		self.visited = False
		self.cost = 123456789.0


class Map:
	def __init__(self):
		rospy.wait_for_service("static_map")
		static_map = rospy.ServiceProxy("static_map", GetMap)
		try:
			self.map = static_map().map
		except:
			print "error receiving map"

		self.pub = rospy.Publisher('/dijkstra_path', Path, queue_size=10)
		rospy.init_node('path_planner', anonymous=True)

		self.xrange = [900,1100]
		self.yrange = [900,1100]

	def getOccupied(self,x,y):
		if self.map.data[(y+self.yrange[0])+(x+self.xrange[0])*self.map.info.width] > 0:
			#print "Obstacle found "+str(y)+ ", " +str(x)+ " | " +str(y+self.yrange[0])+ ", " +str(x+self.xrange[0]) 
			return 1
		elif self.map.data[(y+self.yrange[0])+(x+self.xrange[0])*self.map.info.width] < 0:
			return -1
		else:
			return 0

	def dijkstra(self, a, b, c, d):

		grid = []
		allCells = []
		pq = []
		for x in range(self.xrange[1]-self.xrange[0]):
			grid.append([])
			for y in range(self.yrange[1]-self.yrange[0]):
				cell = Cell(x,y)
				grid[x].append(cell)
				allCells.append((cell.cost,cell))
		
		cell = grid[a][b]
		cell.cost = 0.0
		cell.previous = None
		heapq.heappush(pq,(cell.cost,cell))

		def adjacentCells(cell,grid):
			cells = []
			for pair in [[-1,-1],[-1,0],[-1,1],[0,-1],[0,1],[1,-1],[1,0],[1,1]]:
				if(cell.x + pair[0] < len(grid) and cell.y + pair[1] < len (grid[0])
					and cell.x + pair[0] >= 0 and cell.y + pair[1] >= 0
					and grid[cell.x+pair[0]][cell.y + pair[1]].visited == False):
					cells.append(grid[cell.x+pair[0]][cell.y + pair[1]])
			return cells


		def costFunction(start,end,goal):
			def dist(a,b,c,d):
				return (float(a-c)**2 + float(b-d)**2)**(.5)
			if(self.getOccupied(end.x,end.y) == 1):
				return 9876543.0
			else:
				return dist(start.x,start.y,end.x,end.y) + dist(end.x,end.y,goal.x,goal.y)

		total = 0
		while(len(pq) > 0):
			cell = heapq.heappop(pq)[1]
			#print len(pq)
			cell.visited = True
			#print str(total) + " Cell "+str(cell.x)+", "+str(cell.y) +" Cost "+str(cell.cost) + " Obs "+str(self.getOccupied(cell.x,cell.y))
			total += 1

			if(cell.x == c and cell.y == d):
				print cell.cost
				break

			for adjacent in adjacentCells(cell,grid):
				newCost = cell.cost + costFunction(cell,adjacent,grid[c][d])
				if(newCost < adjacent.cost):
					adjacent.cost = newCost
					adjacent.previous = cell
					heapq.heappush(pq,(adjacent.cost,adjacent))
				

		end = grid[c][d]
		print end.cost
		path = [[end.x,end.y]]
		while(end.previous != None):
			end = end.previous
			path.insert(0,[end.x,end.y])
			
		print path
		return path

	def find_waypoints(self,path):
		waypoints = []
		cur_dir = (0,0)
		for i in range(1,len(path)):
			new_dir = (path[i][0]-path[i-1][0],path[i][1]-path[i-1][1])
			if new_dir != cur_dir:
				waypoints.append(path[i-1])
				cur_dir = new_dir
		waypoints.append(path[len(path)-1])
		return waypoints
				

	def publish_path(self,path):
		poses = []
		for each in path:
			poses.append(PoseStamped(pose = Pose(position = Point(x=each[0],y=each[1]))))
		
		#self.pub.publish(Path(poses = poses))




if __name__ == "__main__":
	m = Map()
	plt.hold(True)
	plt.axis([0,m.xrange[1]-m.xrange[0],0,m.yrange[1]-m.yrange[0]])
	for x in range(m.xrange[1]-m.xrange[0]):
		for y in range(m.yrange[1]-m.yrange[0]):
			if m.getOccupied(x,y) == 1:
				plt.plot(x,y,'b.')
	path = m.dijkstra(90,60,105,150)
	path = m.find_waypoints(path)
	print path
	m.publish_path(path)
	for point in path:
		plt.plot(point[0],point[1],'r.')
	plt.show()