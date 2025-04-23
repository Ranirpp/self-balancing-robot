

Mobile and Autonomous Robots (UE22CS343BB7)
6th Semester
Mini-Project


Project Title: Self Balancing Robot

Team Details:
 Madhu M A
   PES1UG23CS814
Rani 
PES1UG23CS821

Professor Name:
 Adithya Sir

Project Description:  
The Self-Balancing Robot is an autonomous two-wheeled vehicle designed to maintain an upright position without external support. Inspired by the inverted pendulum concept, this project integrates sensors, actuators, and control algorithms to achieve dynamic stability.


Project Objectives:	
The objective of this mini-project is to design, simulate, and implement a two-wheeled self-balancing robot using feedback control systems. The robot should be able to detect its tilt angle using an IMU sensor and apply corrective motor movements in real-time to maintain an upright position.
Specific Objectives:
1.Understand the Dynamics of an Inverted Pendulum System
oModel the robot as an inverted pendulum and study the behavior of unstable systems.
2.Implement Real-Time Sensor Fusion
oUse an MPU6050 IMU sensor to read accelerometer and gyroscope data.
oApply a complementary filter to accurately estimate the robot’s tilt angle.
3.Design and Tune a PID Controller
oCreate a PID-based control system to calculate the correction needed to maintain balance.
oTune the PID gains through experimental testing for optimal performance.
4.Simulate and Test the Robot’s Behavior
oVisualize angle correction and PID response over time using simulation tools or serial data plotting.
5.Integrate Hardware Components (if implemented physically)
oInterface motors, motor drivers, sensors, and power supply to build a working prototype.
6.Demonstrate Autonomous Balancing Capability
oThe final system should maintain balance without user intervention and respond to small disturbances.

Methods and Materials:	

1.System Design:


2.Algorithm/Model Development:
The self-balancing robot mimics the behavior of an inverted pendulum, a dynamic system that is inherently unstable and requires continuous feedback control to remain upright. 
The core of the model development lies in three essential modules:
Modeling the dynamic system (inverted pendulum)
Sensor fusion for accurate tilt angle estimation
Implementation
3.System Modeling;
The self-balancing robot consists of two wheels driven independently by DC motors, with a vertical chassis that supports the electronics and sensors. The balancing task can be abstracted as an inverted pendulum mounted on a mobile base.

4.Implementation Steps:
1.Assemble the chassis and mount the components.
2.Connect the IMU sensor and motor driver to the microcontroller.
3.Implement and test the sensor fusion algorithm.
4.Tune PID controller using trial-and-error method.
5.Test balance and responsiveness under different conditions.
6.Refine code and calibrate control loop timings.

5.Hardware Components (if applicable):
MPU6050 IMU Sensor
Arduino Uno / ESP32
L298N Motor Driver
2 DC Motors with Encoders
Li-ion Battery Pack
Robot Chassis with wheels


6.Software Tools:
Tool	Purpose
Arduino IDE	Writing, uploading, and debugging the microcontroller code for motor control and sensor reading.
Gazebo/simulation	Simulating system dynamics, tuning PID controller, and visualizing response curves.
	
Python (withGazebo)	Visualization of real-time angle data and PID response from serial output.
Fritzing	Designing circuit diagrams for the robot's electronics layout.
GitHub	Version control and code hosting for project collaboration and backups.

Project Outcome:
The self-balancing robot simulation successfully demonstrates the principles of dynamic stability, control systems, and real-time sensor feedback. 
Through a combination of PID control and sensor fusion techniques, the robot is able to maintain an upright position, correcting its posture in response to external disturbances.
 A fully functioning self-balancing robot that maintains stability in place.
 Demonstrated stability under varying tilts and minor pushes.
 Control parameters optimized for fast recovery and minimal oscillation.
	
1.Output results:


2.Simulation video link (drive link)


3.GitHub link (Source code)

References:	
Understanding PID Control – MATLAB/Simulink
https://www.mathworks.com/help/control/ref/pidcontroller.html
Complementary Filter Theory – Bolder Flight Systems
https://www.bolderflight.com/complementary-filter
MPU6050 Datasheet
https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
Inverted Pendulum Dynamics
https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling



