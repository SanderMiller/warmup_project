#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sklearn import linear_model
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import scipy.cluster.hierarchy as hcluster
from scipy.stats import mode
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class personFollowerNode(object):
    def __init__(self):
        rospy.init_node('people_follower')
        rospy.Subscriber('/scan', LaserScan, self.process_lidar)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state = self.identify_person
        self.initialize = True
        self.rate = rospy.Rate(10)
    def process_odom(self,odomMsg):
        #Process Odometry data
        self.current_position = odomMsg.pose.pose

    def process_lidar(self,lidarMsg):
        #print("Updating Lidar")
        self.lidar_data_ranges = np.asarray(lidarMsg.ranges)
        self.lidar_increment = lidarMsg.angle_increment
        if self.initialize == True:
            self.lidar_angles = np.arange(-math.pi,math.pi,self.lidar_increment)
            self.initialize = False
        noninfIndices = np.asarray(np.where(self.lidar_data_ranges != np.inf))[0]
        self.cleanedRanges = np.take(self.lidar_data_ranges, noninfIndices)
        #print(max(self.cleanedRanges))
        self.cleanedAngles = np.take(self.lidar_angles, noninfIndices)
        polar2z = lambda r,ang: r * np.exp( 1j * ang)
        z = polar2z(self.cleanedRanges,self.cleanedAngles)
        self.Y = -1*np.real(z)
        self.X = np.imag(z)
        self.cartPoints = [self.X,self.Y]

    def identify_person(self):
        thresh = 1
        clusters = hcluster.fclusterdata(np.transpose(self.cartPoints), thresh, criterion="distance")
        xAverage = []
        yAverage = []
        for i in range(max(clusters)):
            indices = np.asarray(np.where(clusters == i+1)).astype(int)
            indices = indices[0]
            currentX = np.take(self.cartPoints[0],indices)
            currentY = np.take(self.cartPoints[1],indices)
            xAverage.append(np.average(currentX ))
            yAverage.append(np.average(currentY))
        strongestPersonIndex = mode(clusters).mode[0]-1
        self.strongestY = xAverage[strongestPersonIndex]
        self.strongestX = yAverage[strongestPersonIndex]
        self.goal_heading = self.calc_heading()
        print('Person: '+str(self.strongestX)+', '+str(self.strongestY))
        marker = Marker(
                type=Marker.SPHERE,
                id=0,
                lifetime=rospy.Duration(10),
                pose=Pose(Point(-self.strongestX, -self.strongestY,0), Quaternion(0, 0, 0, 0)),
                scale=Vector3(.6, .6, 0.6),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.vis_pub .publish(marker)
        print('Marking')
        return self.turn
        '''
        plt.scatter(np.transpose(self.cartPoints[0]),np.transpose(self.cartPoints[1]))
        plt.scatter(xAverage, yAverage, c = '#FF0000',marker=(5,2))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Using Average Mass to Identify People')
        plt.show()
        '''
    def turn(self):
        #Turn to match goal heading
        #print('Turning')
        #Convert Quaternion Heading to Euler Angles
        current_euler = euler_from_quaternion([self.current_position.orientation.x,self.current_position.orientation.y,self.current_position.orientation.z,self.current_position.orientation.w])
        #print('Current Heading: ' +str(current_euler))
        #Calculate Heading error
        heading_error = (self.goal_heading - current_euler[2])%math.pi
        #print('Heading: '+str(current_euler[2]))
        #print(heading_error)
        #Generate Proportional Velocity Command
        desired_ang_velocity = heading_error#%math.pi)
        #print(desired_ang_velocity)
        #Send Velocity Command
        self.pub.publish(Twist(angular=Vector3(z=desired_ang_velocity)))
        self.rate.sleep()

        #If the error is less than threshold, move to next state
        if abs(desired_ang_velocity)<.01:
            return self.follow
        else:
            return self.turn
    def follow(self):
        distance = self.lidar_data_ranges[0]
        while distance >.3 and distance !=np.inf:
            #print(distance)
            distance = self.lidar_data_ranges[0]
            self.pub.publish(Twist(linear=Vector3(x=distance)))
            #self.pub.publish(Twist(linear=Vector3(z=distance)))
            self.rate.sleep()
        #print('Stop')
        self.pub.publish(Twist(linear=Vector3(x=0)))
        self.rate.sleep()
        return self.identify_person

    def calc_heading(self):
        #Calculate angle between current and goal position Vectors
        self.pub.publish(Twist(angular=Vector3(z=0)))
        self.current_heading = euler_from_quaternion([self.current_position.orientation.x,self.current_position.orientation.y,self.current_position.orientation.z,self.current_position.orientation.w])
        globalCoords = self.neato_to_world()
        vector1 = np.array([math.tan(self.current_heading[2]),1])
        vector2 = np.array([self.strongestX -self.current_position.position.x,self.strongestY-self.current_position.position.y])#np.array([globalCoords[0][0] -self.current_position.position.x,globalCoords[1][0]-self.current_position.position.y])
        print(vector2.shape)
        angle = math.acos((np.dot(vector1,vector2))/(np.sqrt(vector1.dot(vector1))*np.sqrt(vector2.dot(vector2))))
        angle = angle
       
        if self.strongestX >0:
            angle = 1*angle
            print('Minus Pi')
        angle = angle - self.current_heading[2]
        print("Angle: " + str(angle))

        return angle
    def neato_to_world(self):
        worldCoords = np.asarray([[-self.strongestX*np.cos(self.current_heading[2])],[-self.strongestX*np.sin(self.current_heading[2])],[0]]) +np.asarray([[self.strongestY*(-np.sin(self.current_heading[2]))],[self.strongestY*np.cos(self.current_heading[2])],[0]])+np.asarray([[self.current_position.position.x],[self.current_position.position.y],[1]])
        print('Heading: '+str(self.current_heading[2]))
        print('World: '+str(worldCoords))
        print('Local:' +str(self.current_position.position.x)+' '+str(self.current_position.position.y))
        return worldCoords
    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.state = self.state()


if __name__ == '__main__':
    personFollower  = personFollowerNode()
    personFollower.run()