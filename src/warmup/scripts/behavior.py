#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import thread

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

###############################################################################
#
#
#
###############################################################################
class Wall():
    def __init__(self):
        self.present = False
        self.side = 'left'
        self.contacts = [0,0]

class Obstacle():
    def __init__(self):
        self.present = False
        self.average_dist = 0

class Robot():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.handle_scan, None)
        rospy.init_node('robot', anonymous=True)
        self.ang_vel = .4
        self.linear_vel = 1
        self.msg = Twist()
        self.ranges = []
        self.mode = 'teleop'

        self.degree_offset = 30
        self.dist_to_wall = .5

        self.direction = 1 # 1 = left, -1 = right

    def handle_scan(self, msg):
        """Captures each scan published by the robot"""
        if(len(msg.ranges) < 360):
            print "Error Less than 360 pts: " +str(len(msg.ranges))
        else:
            self.ranges = msg.ranges
        #print self.ranges
    
    def key_catcher(self,args,kwargs):
        """Used to manually force behavior of robot"""
        c = ' '
        while(c != 't'):
            c = getch()
            print c
            if c == 'w':
                self.msg = Twist(linear=Vector3(x=self.linear_vel))
            elif c == 'a':
                self.msg = Twist(angular=Vector3(z=self.ang_vel))
            elif c == 's':
                self.msg = Twist(linear=Vector3(x=-1*self.linear_vel))
            elif c == 'd':
                self.msg = Twist(angular=Vector3(z=-1*self.ang_vel))
            elif c == '1':
                self.mode = 'teleop'
                self.msg = Twist()
            elif c == '2':
                self.mode = 'wall'
                self.msg = Twist()
            elif c == '3':
                self.mode = 'avoid'
                self.msg = Twist()
            else:
                self.msg = Twist()
    
    def find_wall(self):
        wall = Wall()
        
        quadrants = [list(),list(),list(),list()]
        for side in range(2):
            for i in range((90+(180 * side) - self.degree_offset) - 2, 
                (90+(180 * side) - self.degree_offset) +3):
                if self.ranges[i] > 0:
                    quadrants[side*2].append(self.ranges[i])
            for i in range((90+(180 * side) + self.degree_offset) - 2, 
                (90+(180 * side) + self.degree_offset) +3):
                if self.ranges[i] > 0:
                    quadrants[side*2+1].append(self.ranges[i])
        
        if len(quadrants[0]) > 0 and len(quadrants[1]) > 0:
            wall.present = True
            wall.side = 'left'
            wall.contacts[0] = sum(quadrants[0])/float(len(quadrants[0]))
            wall.contacts[1] = sum(quadrants[1])/float(len(quadrants[1]))
        elif len(quadrants[2]) > 0 and len(quadrants[3]) >0:
            wall.present = True
            wall.side = 'right'
            wall.contacts[0] = sum(quadrants[2])/float(len(quadrants[2]))
            wall.contacts[1] = sum(quadrants[3])/float(len(quadrants[3]))
        return wall

    def wall_follow(self):
        wall = self.find_wall()

        if self.find_obstacle().present and wall.present:
            if wall.side == "right":
                self.msg = Twist(angular=Vector3(z=1*self.ang_vel))
            else:
                self.msg = Twist(angular=Vector3(z=-1*self.ang_vel))

        elif wall.present:
            ang_vel = ((wall.contacts[0] - wall.contacts[1])*1)
            self.msg = Twist(linear=Vector3(x=.3),angular=Vector3(z=ang_vel))
            
        else:
            self.mode = 'avoid'
            print "No wall"


    def leagacy_wall_follow(self):
        valid_forward = []
        valid_reverse = []
        right = 1
        for i in range(43+right*180,48+right*180):
            #print 'ranges'+str(i) +str(self.ranges[i])
            if self.ranges[i] > 0:
                valid_forward.append(self.ranges[i])
        for i in range(133+right*180,138+right*180):
            #print 'ranges'+str(i) +str(self.ranges[i])
            if self.ranges[i] > 0:
                valid_reverse.append(self.ranges[i])

        if len(valid_reverse) > 0 and len(valid_forward) > 0:
            forward_av = sum(valid_forward)/float(len(valid_forward))
            reverse_av = sum(valid_reverse)/float(len(valid_reverse))
            print 'forw: '+ str(forward_av) +' revr: ' + str(reverse_av) + ' diff: '+str((forward_av - reverse_av)*1)
            ang_vel = ((forward_av - reverse_av)*1)
            self.msg = Twist(linear=Vector3(x=.3),angular=Vector3(z=ang_vel))
            
        else:
            self.mode = 'teleop'
            self.msg = Twist()
            print "No wall"

    def find_obstacle(self):
        obstacle = Obstacle()
        valid_obstacles = []
        left_points = 0
        right_points = 0

        for i in range(-35,36):
            if self.ranges[i] != 0 and self.ranges[i] < 1:
                valid_obstacles.append(self.ranges[i])
                if i>0:
                    left_points += 1
                elif i<0:
                    right_points += 1

        if len(valid_obstacles) > 0:
            obstacle.present = True
            obstacle.average_dist = sum(valid_obstacles)/float(len(valid_obstacles))
            if left_points > right_points:
                self.direction = -1 #right
            elif right_points > left_points:
                self.direction = 1 #left

        return obstacle


    def object_avoidance(self):
        if self.find_wall().present:
            print "Found Wall"
            self.mode = "wall"
            return

        obstacle = self.find_obstacle()
        if obstacle.present:  
            lin_vel = (obstacle.average_dist) * .2
            ang_vel = (self.direction * 1)/obstacle.average_dist
            self.msg = Twist(linear=Vector3(x=lin_vel),angular=Vector3(z=ang_vel))
        else:
            self.msg = Twist(linear=Vector3(x=self.linear_vel))
            #print "No obstacles"


    def run(self):
        thread.start_new_thread(self.key_catcher,(None,None))
        r=rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode == 'wall':
                self.wall_follow()
            elif self.mode == 'avoid':
                self.object_avoidance()
            self.pub.publish(self.msg)
            r.sleep()
            
        
if __name__ == '__main__':
    try:
        r = Robot()
        r.run()
    except rospy.ROSInterruptException: pass