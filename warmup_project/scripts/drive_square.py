#!/usr/bin/env python3
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospy
import math
import numpy as np

class drive_square_node():
    def __init__(self):
        #Initialize various Variables
        self.inititialize = True
        self.state = self.calc_goal
        self.side = 1

        #Initialize ROS publisher and subscriber
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        rospy.init_node('driveSquare', anonymous=True)
        self.rate = rospy.Rate(10)

    def process_odom(self,odomMsg):
        #Process Odometry data
        self.current_position = odomMsg.pose.pose
        
    def calc_goal(self):
        #Calculate Goal Position and Heading

        self.init_pos = self.current_position
        self.side = self.side % 4
        #Calculate next Goal Point Based on Side length
        if self.side == 1:
            self.goal_pos = [self.init_pos.position.x+1, self.init_pos.position.y, self.init_pos.position.z]
            
        elif self.side == 2:
            self.goal_pos = [self.init_pos.position.x, self.init_pos.position.y+1, self.init_pos.position.z]
         
        elif self.side == 3:
            self.goal_pos = [self.init_pos.position.x-1, self.init_pos.position.y, self.init_pos.position.z]
          
        elif self.side == 0:
            self.goal_pos = [self.init_pos.position.x, self.init_pos.position.y-1, self.init_pos.position.z]
            
        #Convert Quaternion Heading to Euler angles
        self.current_heading = euler_from_quaternion([self.init_pos.orientation.x,self.init_pos.orientation.y,self.init_pos.orientation.z,self.init_pos.orientation.w])
        
        #Calculate Goal Heading
        self.goal_heading = self.calc_heading()
        self.inititialize = False

        return self.turn

    def calc_heading(self):
        #Calculate angle between current and goal position Vectors
        vector1 = np.array([math.tan(self.current_heading[2]),1])
        vector2 = np.array([self.goal_pos[0]-self.init_pos.position.x,self.goal_pos[1]-self.init_pos.position.y])
        angle = math.acos((np.dot(vector1,vector2))/(np.sqrt(vector1.dot(vector1))*np.sqrt(vector2.dot(vector2))))
        angle = (angle-math.pi/2)
        if self.side>2:
            angle = angle-math.pi/2
        if self.side ==1 and self.inititialize == False:
            angle = angle+math.pi/2

        return angle


    def turn(self):
        #Turn to match goal heading

        #Convert Quaternion Heading to Euler Angles
        current_euler = euler_from_quaternion([self.current_position.orientation.x,self.current_position.orientation.y,self.current_position.orientation.z,self.current_position.orientation.w])

        #Calculate Heading error
        heading_error = (self.goal_heading - current_euler[2])

        #Generate Proportional Velocity Command
        desired_ang_velocity = abs(heading_error%math.pi)

        #Send Velocity Command
        self.pub.publish(Twist(angular=Vector3(z=desired_ang_velocity)))
        self.rate.sleep()

        #If the error is less than threshold, move to next state
        if desired_ang_velocity<.01:
            return self.move_forward
        else:
            return self.turn

    def move_forward(self):

        #Calculate Position Error
        pos_error = max(abs(np.array(self.goal_pos)-np.array([self.current_position.position.x,self.current_position.position.y, self.current_position.position.z])))
        #Calculate Desired Velocity (Gain=1)
        desired_velocity = pos_error

        #Publish Linear Velocity
        self.pub.publish(Twist(linear=Vector3(x=desired_velocity)))
        self.rate.sleep()
        
        #If the error is less than threshold, move to next State
        if abs(pos_error)<0.1:
            self.pub.publish(Twist(linear=Vector3(x=0)))
            self.side+=1
            return self.calc_goal
        else:
            return self.move_forward

    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.state = self.state()


if __name__ == '__main__':
    square = drive_square_node()
    square.run()