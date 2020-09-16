#!/usr/bin/env python3

import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist, Vector3
import rospy

#Create teleop Node
class teleop_node():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('teleop', anonymous=True)
        self.rate = rospy.Rate(10)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        key = None
        velocity = Twist()

        while key != '\x03':
            velocity.linear = Vector3(0,0,0)
            velocity.angular = Vector3(0,0,0)
            key = self.getKey()
            print(key)
            
            if key == 'w':
                velocity.linear = Vector3(1,0,0)
            elif key == 's':
                velocity.linear = Vector3(-1,0,0)

            if key == 'a':
                velocity.angular = Vector3(0,0,1)
            elif key == 'd':
                velocity.angular = Vector3(0,0,-1)

            self.pub.publish(velocity)
            self.rate.sleep()
    

if __name__ == '__main__':
    tele = teleop_node()
    tele.run()
