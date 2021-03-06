# Comp Robo Warmup Project

<h2>Robot Teleop</h2>

In order to control the robot's movement I wrote a simple teleoperation script that mapped keystrokes to linear and angular velocities. These commanded velocities are published to the <code>cmd_vel</code> ROS topic.
Key | Command
------------ | -------------
W | Forward
S | Reverse
A |Turn Left
D | Turn Right
Q | Stop


<h2>Driving in a Square</h2>

In order to drive in a square, I explored the robot's odometry data. I used a finite state controller and proportional control in order to calculate and move to the next goal position. 
<p align="center">
  <img width="460" height="300" src="warmup_project/screenshots/SquareStateController.jpg">
</p>
<h3>Calculating Goal Point and Position</h3>
This state is called when the robot has reached a corner of the square. Based on what side of the square the robot has moved along, and the current position of the robot using the odometry data, a goal point is calculated. This is either the robot's X position ± 1 or the robot's Y position ± 1. Once the next waypoint is calculated, the heading from the current position to the goal position is calculated using the formula <img src="https://render.githubusercontent.com/render/math?math=cos(\theta) = a \cdot b/||a||\cdot ||b||"> where a and b are the current and goal positions. Factors of π/2 were added/subtracted as needed to account for the robot's coordinate system.
<h3>Turning</h3>
Once the goal position and angle have been calculated we enter the turning state. In order to proceed to the next waypoint the robot must turn so that its current heading is equal to that of the goal heading. I implemented a simple proportional controller in order to calculate the angular velocity command to be sent to the robot. The error is calculated as the difference between the goal heading and current heading. While the error is above a threshold value, I used 0.1 radians, the robot remains in the turning state and continues turning at an angular velocity equal to that of the error. Once the error is below the threshold, the robot enters the "Moving Forward" state.
<h3>Moving Forward</h3>
Once the robot is in the moving forward state it is facing the goal point, and just needs to move forward in order to reach it. To do this I again used a proportional controller, where the error is the distance between the robot and the goal point. As the robot approaches the waypoint it's velocity decreases proportionally. For both the moving forward and turning controllers, I found a gain value of 1 worked just fine.

<h3>Results, Reflections, and Next Steps</h3>
<p align="center">
  <img width="600" height="400" src="warmup_project/screenshots/DriveSquare.gif">
  
I am very pleased with my results. The square was fairly concise, and even after letting the script run for a few minutes the robot remained in relatively the same position. There was a little bit of drift over time but this is to be expected. This could be slightly mitigated by decreasing the threshold values for both proportional controllers. 
Identical behaviour could have been replicated much more concisely with using purely time based controls; however, I think I learned a lot more about the robot's coordinate system and odometry data, finite state controllers, and proportional control through my implementation.
<h2>Wall Following</h2>
To generate wall following behaviour I made use of the Hough Transform and again used a finite state controller. The goal of the behaviour was to be able to identify a wall from any orientation, approach it, and drive parallel to it at a distance of 1m.


<p align="center">
  <img width="600" height="400" src="warmup_project/screenshots/WallFollowingStateDiagram.jpg">

<h3>Identifying the Wall</h3>
Identifying the wall is probably the most complex aspect of this behaviour. In order to do this I made use of the Hough Transform. The Hough Transform is an algorithm commonly used in computer vision to identify lines within a binary image, often used in tandem with edge detection operators such as the Canny edge detector. The way the Hough transform works is by essentially spinning a line around each point, π radians. At each orientation on each point, this line has a unique angle and offset value, theta and rho. Based on the number of other points this line passes through it is given a value, and lines that pass through more than some threshold number of points are identified as lines. Since the lidar data is relatively sparse I used a threshold value of 4 points to characterize a line/wall. 
I began by subscribing to the lidar topic and because I used OpenCV's version of the Hough Transform I interpolated the data into a 1000x1000 binary image.
<p align="center">
  <img width="1300" height="400" src="warmup_project/screenshots/Merged2WallsLabeled.png">
  
  From there, I used the <code>cv2.HoughLines()</code> function on the inverted binary image. Below visualizes all possible lines consisting of 4 or more points.
 <p align="center">
  <img width="500" height="500" src="warmup_project/screenshots/AllLines.png">
  
 That's a lot of lines despite the fact there are only two walls within view of the robot's lidar. In order to get rid of excess noise I filtered the lines using a technique known as hierarchical clustering. As you can see, even though there are many lines detected, there are two main groups or clusters that reflect the two walls. When viewing these lines in the Hough space this is even more clear. By using hierarchical clustering I was able to group the lines together based on their euclidean distance apart in a normalized Hough space. In order to normalize the Hough space, I interpolated the theta values to have a range of 1000, rather than pi. This helps to differentiate between two walls with similar rho values but different theta values. Once the clusters were formed I took the average rho and theta values for each cluster and used those to represent the wall. 
  <p align="center">
  <img width="500" height="500" src="warmup_project/screenshots/FinalScatterPlot.png">
   
  The average rho and theta values, given by the red start in the scatter plot, are represented in the image below. As you can see they correlate pretty well with the actual two walls.
   <p align="center">
  <img width="500" height="500" src="warmup_project/screenshots/AveragedLines.png">
  
 Although for most of the wall following module I only dealt with one wall at a time, it was very helpful to be able to distinguish between walls, and isolate values associated with a particular wall. Also, I think this technique of analysis in the Hough space could prove useful for a wide variety of future applications.
 Now that we have a rho and theta value for each wall, it's time to choose which wall to follow. I choose which wall to follow based on the number of lines within it's Hough space cluster. This *should* pick the wall with the most measured lidar ranges. Once a wall has been identified, the robot enters the turning state.
 
 <h3>Turning to Approach the Wall</h3>
 Once the wall has been identified, the robot turns in order to position itself 1 meter away from the wall. To do this, the robot must turn perpendicular to the wall so that it is directly facing the wall. To do this I again used a proportional controller, where it would continue turning while the absolute value of the error in heading, calculated by the goal heading minus the current heading, was above some threshold value. Once the error dipped below the threshold value, the robot would move on to the next state.
 <h3>Moving to Approach the Wall</h3>
 Once the robot has identified and faced the wall, it now approaches. I again used proportional control based on the range value directly ahead of the robot. As that value approached a distance of 1, the robot slowed down. Once the robot had come to a complete stop, it would now turn parallel to the wall in order to follow it.
  <h3>Turning to Follow the Wall</h3>
  Similarly to the first turning state, the robot used proportional control to approach a goal heading, only this time the heading was parallel to the wall. When the robot finally reached that point, it would move on to the wall following state.
  <h3>Moving to Follow the Wall</h3>
  Once the robot is facing parallel to the wall, with a 1 meter offset, it simply has to go forward in order to follow the wall. It also ends up keeping track of the lidar ranges 90 degrees to the righ and left of itself to ensure it does not stray too far from the wall. If this range perpendicular to the wall becomes greater or less than 1±0.5, the robot returns to the identifying wall state in order to adjust itself and the process starts over again.
  <h3>Results and Reflections</h3>
  <p align="center">
  <img width="500" height="500" src="warmup_project/screenshots/Wall_follower.gif">
  
  I put a lot of time and effort into making this behaviour work, and it ended up working okay; however, it was very inconsistent, sometimes it worked great and other times not so well. I think there was something a little bit off with the conversion between the wall angle theta, and the robots coordinate system. In the future it would be nice to take another look at this and see if it can be fixed. Also, the computational power of constantly checking for a new wall made the movement and turning a little choppy.
  
 <h2>Person Following</h2>
 In order to identify and follow people I again used hierarchical clustering, but this time on lidar data to distinguish between multiple objects, and calculate the center of mass of each object. Then, that center of mass would become a goal position to which the robot would drive towards. Yet again I used a finite state controller to imitate the person following behaviour.
 <p align="center">
  <img width="450" height="300" src="warmup_project/screenshots/PersonFollowerStateDiagram.jpg">
  <h3>Identifying a Person</h3>
  In order to identify discrete objects or people, I divided the measured lidar ranges into clusters using hierarchical clustering. Then, for each cluster I calculate the center of mass by taking the average X and Y coordinate of each point within a given cluster. This process is shown below.
   <p align="center">
  <img width="900" height="400" src="warmup_project/screenshots/MergedMass.png">
  
  The average mass of the cluster with the most points is chosen to be our goal position. This coordinate is with respect to the robot's coordinate system. In order to calculate the proper headin between the robots world coordinate position, and the location of the person, we must first calculate the position of the person with respect to the world's coordinate system. To do this I multiplied the 3x3 rotation matrix where theta was the robot's heading, concatenated with the robot's global position, by a 3x1 column vector of the person's position with respect to the robot. Now that I had both the robot and person's global position I could calculate the ideal heading using <img src="https://render.githubusercontent.com/render/math?math=cos(\theta) = a \cdot b/||a||\cdot ||b||"> 
  <h3>Turning to Face the Person</h3>
  Once the ideal heading had been calculated I again used a proportional controller to minimize the error between the robot's current heading and the heading to point at the identified person.
  <h3>Approaching the Person</h3>
  I used the lidar's front sensor range to measure the distance between the robot and the person as it followed. Again I used proportional control with relation to distance from the person. I set a distance threshold value of 0.25 meters to stop the robot from running into the person. Once within the specified distance the robot would return to the identifying people state and continue the cycle.
  <h3>Results and Reflections</h3>
  I think that the technique I used is fairly robust. Using the clustering algorithm to isolate different objects is very useful for ensuring the average mass is calculated using only lidar ranges for that particular object. I do think something was off with my conversion from robot to world coordinate systems. The script worked great upon initialization, when the robot and world coordinates were aligned. After reaching the person the first time however, the robot had some issues that I think could be fixed by updating the coordinate transformation.
  <h2>Obstacle Avoidance</h2>
  Unfortunately I did not get a chance to implement obstacle avoidance, however I think I could have used a strategy similar to that used for person following. By using the clustering technique on the lidar data I could isolate each distinct object. Then, by adding up all the vectors from each object's global position to mine I could get a net obstacle vector. Then by adding that vector with the vector from the robot to the goal position I would get my ideal heading. By constantly updating this information as I move I could reactively avoid obstacles as they appear on the lidar while also approaching my goal position.
