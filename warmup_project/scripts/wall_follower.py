#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from sklearn import linear_model
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import scipy.cluster.hierarchy as hcluster

class wall_follower():
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_lidar)
        self.initialize = True
        self.state = self.identify_wall

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
       
    def identify_wall(self):
        print("Identifying Wall--------")
        img = 255*np.zeros([1000,1000])
        interp = interp1d([-5,5],[0,1000])
        imgPointsX = np.rint(interp(self.X))
        imgPointsY = np.rint(interp(self.Y))
        imgPointsX = imgPointsX.astype(int)
        imgPointsY = imgPointsY.astype(int)
        img = img.astype(np.ubyte)
        for i in range(imgPointsX.size):
            img[imgPointsX[i]][imgPointsY[i]] = 255
        imgInv = cv.bitwise_not(img)
        lines = cv.HoughLines(img,1,np.pi/180,4)#,10,10)
        #print(lines.shape)
        blankim = cv.merge((imgInv, imgInv, imgInv))
        
        rhoVals = []
        thetaVals = []
        
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
            print(averagedData[i][0])
            
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
        
        
        


    def approach_wall(self):
        
        return self.follow_wall
        
    def follow_wall(self):
        
        return self.identify_wall
        

    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            self.state = self.state()


if __name__ == '__main__':
    follower = wall_follower()
    follower.run()