#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sklearn import linear_model
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import scipy.cluster.hierarchy as hcluster
from scipy.stats import mode
from tf.transformations import euler_from_quaternion

class wall_follower():
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_lidar)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.rate = rospy.Rate(10)
        self.initialize = True
        self.state = self.identify_wall
        self.heading_error = 10
        self.turn_toward = True

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
        self.X = -1*np.real(z)
        self.Y = np.imag(z)
        self.cartPoints = [self.X,self.Y]
       
    def process_odom(self,odomMsg):
        self.current_position = odomMsg.pose.pose
        self.current_heading_euler = euler_from_quaternion([self.current_position.orientation.x,self.current_position.orientation.y,self.current_position.orientation.z,self.current_position.orientation.w])

    def identify_wall(self):
        #print("Identifying Wall--------")
        img = 255*np.zeros([1000,1000])
        interp = interp1d([-5,5],[0,1000])
        imgPointsX = np.rint(interp(self.X))
        imgPointsY = np.rint(interp(self.Y))
        imgPointsX = imgPointsX.astype(int)
        imgPointsY = imgPointsY.astype(int)
        img = img.astype(np.ubyte)
        for i in range(imgPointsX.size):
            img[imgPointsX[i-3]][imgPointsY[i-3]] = 255
        imgInv = cv.bitwise_not(img)
        lines = cv.HoughLines(img,1,np.pi/180,4)#,10,10)
        #print(lines.shape)
        blankim = cv.merge((imgInv, imgInv, imgInv))
        
        rhoVals = []
        thetaVals = []
        try:
            for i in range(lines.shape[0]):
                for rho,theta in lines[i]:
                    rhoVals.append(rho)
                    thetaVals.append(theta)
                    
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))

                    cv.line(blankim,(x1,y1),(x2,y2),(0,0,255),2)
        except AttributeError:
            print("No Walls detected")
            self.turn_toward == True
            return self.identify_wall
                

        interpTheta = interp1d([0,np.pi],[-1000,1000])
        thetaInterpolated = interpTheta(thetaVals)
        data = np.transpose(np.stack((rhoVals,thetaInterpolated)))
        thresh = 50
        clusters = hcluster.fclusterdata(data, thresh, criterion="distance")
        clusteredData = np.column_stack((data,clusters))
        #print(clusteredData.shape)
        rhoAverage = []
        thetaAverage = []
        for i in range(max(clusters)):
            indices = np.asarray(np.where(clusters == i+1)).astype(int)
            indices = indices[0]
            currentRho = np.take(rhoVals,indices)
            currentTheta = np.take(thetaVals,indices)
            rhoAverage.append(np.average(currentRho))
            thetaAverage.append(np.average(currentTheta))

        averagedData = np.column_stack((rhoAverage,thetaAverage))
        for i in range(averagedData.shape[0]):
            #print(averagedData[i][0])
            
            for rho,theta in np.reshape(averagedData[i],(1,2)):
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                cv.line(blankim,(x1,y1),(x2,y2),(255,0,0),2)
        strongestWallIndex = mode(clusters).mode[0]-1
        self.strongestRho, self.strongestTheta = averagedData[strongestWallIndex][:]
        #print(self.strongestRho, self.strongestTheta)
        '''
        for i in range(imgPointsX.size):
            blankim[imgPointsX[i]][imgPointsY[i],:] = 0
        
        
        plt.scatter(rhoVals,thetaVals,c=clusters)
        plt.scatter(rhoAverage, thetaAverage, c = '#FF0000',marker=(5,2))

        plt.xlabel('Rho')
        plt.ylabel('Theta')
        plt.title('Using Hierarchical Clustering to Filter Hough Lines')
        plt.show()
        
        cv.imshow('image', blankim)
        cv.waitKey(0)
        cv.destroyAllWindows()
        '''
        print('Rho:' +str(thetaAverage))
        return self.turn

    def turn(self):
        self.last_error = self.heading_error 
        if self.turn_toward == True:
            print('Turning to Approach')
            self.heading_error = (1/(5*((math.pi/2) -self.strongestTheta)))+math.pi/2#1/(5*((math.pi/2) -self.strongestTheta+math.pi/2))
            desired_ang_velocity = self.heading_error
        else:
            print('Turning to Follow Wall')
            self.heading_error = 1/(2*((math.pi/2) -self.strongestTheta))
            desired_ang_velocity = self.heading_error+self.last_error
        
        print("Velocity:"+str(desired_ang_velocity))
        self.pub.publish(Twist(angular=Vector3(z=desired_ang_velocity)))
        self.rate.sleep()

        if abs(desired_ang_velocity )<.2 and self.turn_toward == True and self.lidar_data_ranges[0]!=np.inf:
            self.pub.publish(Twist(linear=Vector3(x=0.5)))    
            self.pub.publish(Twist(angular=Vector3(0,0,0)))
            self.rate.sleep()
            return self.approach_wall
        elif abs(desired_ang_velocity )<.2 and self.turn_toward == True:
            self.pub.publish(Twist(angular=Vector3(0,0,.5)))
            rospy.sleep(3)
            return self.identify_wall
        elif abs(desired_ang_velocity )<.2 and self.turn_toward == False:
            self.pub.publish(Twist(linear=Vector3(x=0.5)))
            self.pub.publish(Twist(angular=Vector3(0,0,0)))
            self.rate.sleep()
            return self.follow_wall
        else:
            print('Stop')
            self.pub.publish(Twist(linear=Vector3(x=0)))
            self.rate.sleep()
            return self.identify_wall



    def approach_wall(self):
        print('Approaching Wall')
        offset_error = min(self.lidar_data_ranges[0]-1,self.lidar_data_ranges[181]-1)
        while(abs(offset_error)>.01):
            offset_error = min(self.lidar_data_ranges[0]-1,self.lidar_data_ranges[181]-1)
            if self.lidar_data_ranges[181]-1<self.lidar_data_ranges[0]-1:
                offset_error = -offset_error
            self.pub.publish(Twist(angular=Vector3(z=0)))
            self.pub.publish(Twist(linear=Vector3(x=offset_error)))
            self.rate.sleep()
        self.pub.publish(Twist(linear=Vector3(x=0)))
        self.turn_toward = False
        self.heading_error = 10
        return self.identify_wall
        
    def follow_wall(self):
        print('Following Wall')
        offset_error = min(abs(self.lidar_data_ranges[91]-1), abs(self.lidar_data_ranges[271]-1))
        while offset_error<0.5:
            offset_error = min(abs(self.lidar_data_ranges[91]-1), abs(self.lidar_data_ranges[271]-1))
            self.pub.publish(Twist(linear=Vector3(x=.5)))
            self.pub.publish(Twist(angular=Vector3(z=0)))
        self.pub.publish(Twist(linear=Vector3(x=.5)))
        self.turn_toward = True
        return self.identify_wall
        

    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.state = self.state()


if __name__ == '__main__':
    follower = wall_follower()
    follower.run()