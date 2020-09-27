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
This state is called when the robot has reached a corner of the square. Based on what side of the square the robot has moved along, and the current position of the robot using the odometry data, a goal point is calculate. This is either the robot's X position ± 1 or the robot's Y position ± 1. Once the next waypoint is calulated, the heading from the current position to the goal position is calculated using the formula <img src="https://render.githubusercontent.com/render/math?math=cos(\theta) = a \cdot b/||a||\cdot ||b||">
<h3>Turning</h3>
<h3>Moving Forward</h4>

<p align="center">
  <img width="342" height="342" src="warmup_project/screenshots/DriveSquare.gif">
