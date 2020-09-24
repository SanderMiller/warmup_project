#!/usr/bin/env python3
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, ColorRGBA
import rospy
import math
import numpy as np

class drive_square_node():
    def __init__(self):
        self.state = self.calc_goal
        self.side = 1
        print('Init')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        rospy.init_node('driveSquare', anonymous=True)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        self.inititialize = True
        self.rate = rospy.Rate(10)

    def process_odom(self,odomMsg):
        self.current_position = odomMsg.pose.pose
        
    
    def calc_goal(self):
        self.init_pos = self.current_position
        self.side = self.side % 4
        print("Side: " + str(self.side)+'---------------')
        print("Initial Position: " +str(self.init_pos.position))

        if self.side == 1:
            self.goal_pos = [self.init_pos.position.x+1, self.init_pos.position.y, self.init_pos.position.z]
            print("Real Goal Angle = 0")
        elif self.side == 2:
            self.goal_pos = [self.init_pos.position.x, self.init_pos.position.y+1, self.init_pos.position.z]
            print("Real Goal Angle = 1.577")
        elif self.side == 3:
            self.goal_pos = [self.init_pos.position.x-1, self.init_pos.position.y, self.init_pos.position.z]
            print("Real Goal Angle = 3.14")
        elif self.side == 0:
            self.goal_pos = [self.init_pos.position.x, self.init_pos.position.y-1, self.init_pos.position.z]
            print("Real Goal Angle = -1.577")
        marker = Marker(
                type=Marker.SPHERE,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(self.goal_pos[0],self.goal_pos[1],self.goal_pos[2]), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.0),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                )
        self.marker_publisher.publish(marker)
        print("Goal Point: "+str(self.goal_pos))

        print('calc')
        self.current_heading = euler_from_quaternion([self.init_pos.orientation.x,self.init_pos.orientation.y,self.init_pos.orientation.z,self.init_pos.orientation.w])
        
        self.goal_heading = self.calc_heading()#self.current_heading[2]-(math.pi/2)#%(math.pi/2)
        print("Calculated Goal Heading: "+str(self.goal_heading))
        print("Current Heading: "+str(self.current_heading))
        self.inititialize = False
        return self.turn
    def calc_heading(self):

        vector1 = np.array([math.tan(self.current_heading[2]),1])
        vector2 = np.array([self.goal_pos[0]-self.init_pos.position.x,self.goal_pos[1]-self.init_pos.position.y])
        angle = math.acos((np.dot(vector1,vector2))/(np.sqrt(vector1.dot(vector1))*np.sqrt(vector2.dot(vector2))))
        angle = -1*(angle-math.pi/2)
        if self.side>2:
            angle = angle-math.pi/2
        if self.side ==1 and self.inititialize == False:
            angle = angle+math.pi/2

        return angle


    def turn(self):
        #print('Turn')
        current_euler = euler_from_quaternion([self.current_position.orientation.x,self.current_position.orientation.y,self.current_position.orientation.z,self.current_position.orientation.w])
        #print(current_euler)
        #print(self.current_heading[2]+(math.pi/2))
        heading_error = self.goal_heading - current_euler[2]
        #print("Head Error: " +str(heading_error))
        desired_ang_velocity = heading_error
        self.pub.publish(Twist(angular=Vector3(z=desired_ang_velocity)))
        self.rate.sleep()
        if abs(heading_error)<.01:
            return self.move_forward
        else:
            return self.turn

    def move_forward(self):
        #Calculate error
        #print('Move')
        
        self.current_position.position.x
        pos_error = max(abs(np.array(self.goal_pos)-np.array([self.current_position.position.x,self.current_position.position.y, self.current_position.position.z])))
        #Proportional Controller for forward velocity
        desired_velocity = 1*pos_error
        self.pub.publish(Twist(linear=Vector3(x=desired_velocity)))
        self.rate.sleep()
        
        #print('posError '+str(pos_error))
        if abs(pos_error)<0.1:
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