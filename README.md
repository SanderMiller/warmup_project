# Comp Robo Warmup Project
This is the base repo for the Olin Computational Robotics warmup project.

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
This state is called when the robot has reached a corner of the square. Based on what side of the square the robot has moved along, and the current position of the robot using the odometry data, a goal point is calculate. This is either the robot's X position ± 1 or the robot's Y position ± 1. Once the next waypoint is calculated, the heading from the current position to the goal position is calculated using the formula <img src="https://render.githubusercontent.com/render/math?math=cos(\theta) = a \cdot b/||a||\cdot ||b||"> where a and b are the current and goal positions. Factors of π/2 were added/subtracted as needed to account for the robot's coordinate system.
<h3>Turning</h3>
Once the goal position and angle have been calculated we enter the turning state. In order to proceed to the next waypoint the robot must turn so that its current heading is equal to that of the goal heading. I implemented a simple proportional controller in order to calculate the angular velocity command to be sent to the robot. The error is calculated as the difference between the goal heading and current heading. While the error is above a threshold value, I used 0.1 radians, the robot remains in the turning state and continues turning at an angular velocity equal to that of the error. Once the error is below the threshold, the robot enters the "Moving Forward" state.
<h3>Moving Forward</h3>
Once the robot is in the moving forward state it is facing the goal point, and just needs to move forward in order to reach it. To do this I again used a proportional controller, where the error is the distance between the robot and the goal point. As the robot approaches the waypoint it's velocity decreases proportionally. For both the moving forward and turning controllers, I found a gain value of 1 worked just fine.

<h3>Results, Reflections, and Next Steps</h3>
<p align="center">
  <img width="600" height="400" src="warmup_project/screenshots/DriveSquare.gif">
  
I am very pleased with my results. The square was fairly concise, and even after letting the script run for a few minutes the robot remained in relatively the same position. There was a little bit of drift over time but this is to be expects. This could be slightly mitigated by decreasing the threshold values for both proportional controllers. 
Identical behaviour could have been replicated much more concisely with using purely time based controls; however, I think I learned a lot more about the robot's coordinate system and odometry data, finite state controllers, and proportional control through my implementation.
<h2>Wall Following</h2>
To generate wall following behaviour I made use of the Hough Transform and again used a finite state controller. The goal of the behaviour was to be able to identify a wall from any orientation, approach it, and drive parallel to it at a distance of 1m.


<p align="center">
  <img width="600" height="400" src="warmup_project/screenshots/WallFollowingStateDiagram.jpg">

<h3>Identifying the Wall</h3>
Identifying the wall is probably the most complex aspect of this behaviour. In order to do this I made use of the Hough Transform. The Hough Transform is an algorithm commonly used in computervision to identify lines within a binary image, often used in tandem with edge detection operators such as the Canny edge detector. The way the Hough transform works is by essentially spinning a line around each point, π radians. At each orientation on each point, this line has a unique angle and offset value, theta and rho. Based on the number of other points this line passes through it is given a value, and lines that pass through more than some threshold number of points are identified as lines. Since the lidar data is relatively sparse I used a threshold value of 4 points to characterize a line/wall. 
I began by subscribing to the lidar topic and because I used openCV's version of the Hough Transform I interpolated the data into a 1000x1000 binary image.
<p align="center">
  <img width="1300" height="400" src="warmup_project/screenshots/Merged2WallsLabeled.png">
  
  From there, I used the <code>HoughLines()</code> function on the inverted binary image. Below visualized all lines consisting of 4 or more points.
 <p align="center">
  <img width="500" height="500" src="warmup_project/screenshots/AllLines.png">
