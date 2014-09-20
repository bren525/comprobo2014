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

class Robot():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.handle_scan, None)
        rospy.init_node('robot', anonymous=True)
        self.turn_vel = .4
        self.linear_vel = 1
        self.msg = Twist()
        self.ranges = []
        self.mode = 'teleop'

        self.direction = 1 # 1 = left, -1 = right

    def handle_scan(self, msg):
        """Captures each scan published by the robot"""
        self.ranges = msg.ranges
        print self.ranges
    
    def key_catcher(self,args,kwargs):
        """Used to manually force behavior of robot"""
        c = ' '
        while(c != 't'):
            c = getch()
            print c
            if c == 'w':
                self.msg = Twist(linear=Vector3(x=self.linear_vel))
            elif c == 'a':
                self.msg = Twist(angular=Vector3(z=self.turn_vel))
            elif c == 's':
                self.msg = Twist(linear=Vector3(x=-1*self.linear_vel))
            elif c == 'd':
                self.msg = Twist(angular=Vector3(z=-1*self.turn_vel))
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
    
    def check_for_wall(self):
        wall = Wall()
        quadrants = [[],[],[],[]]
        for contact in range(4):
            for i in range(45*wall - 2, 45*wall + 3):
                if self.ranges[i] > 0:
                    quadrants[contact].append(self.ranges(i))
        if len(uadrants[0]) > 0 and len(quadrants[1]) > 0:
            wall.side = left
            wall.quadrants
        return wall

    def wall_follow(self):
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
            print "No wall"

    def object_avoidance(self):


        valid_obstacles = []
        left_points = 0
        right_points = 0

        for i in range(-20,21):
            if self.ranges[i] != 0:
                valid_obstacles.append(self.ranges[i])
                if i>0:
                    left_points += 1
                elif i<0:
                    right_points += 1

        if len(valid_obstacles) > 0:
            if left_points > right_points:
                self.direction = -1 #right
            elif right_points > left_points:
                self.direction = 1 #left
            #print "right" if direction == -1 else "left"
            print str(left_points) + "," +str(right_points)
            average_dist = sum(valid_obstacles)/float(len(valid_obstacles))
            lin_vel = (average_dist - .2) * .2
            ang_vel = (5 - average_dist) * self.direction * .4
            self.msg = Twist(linear=Vector3(x=lin_vel),angular=Vector3(z=ang_vel))
        else:
            self.msg = Twist(linear=Vector3(x=self.linear_vel))
            print "No obstacles"


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