# Wall-e-simulation-ros2
A simulation of a self-balancing and line following bot using ROS2

## Table of Contents
* [About the Project](#about-the-project)
  * [Tech Stack](#tech-stack)
  * [Tools](#tools)
  * [File Structure](#file-structure)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
  * [Configuration](#configuration)
* [Usage](#usage)
* [Theory and Approach](#theory-and-approach)
* [Results and Demo](#results-and-demo)
* [Future Work](#future-work)
* [Troubleshooting](#troubleshooting)
* [Contributors](#contributors)
* [Acknowledgements and Resources](#acknowledgements-and-resources)
* [License](#license)

## About the Project

![Screenshot from 2021-10-16 01-21-54](https://user-images.githubusercontent.com/82901720/137546447-a77caa4a-a804-49ad-93dc-8da561bad422.png)
![Screenshot from 2021-10-16 01-22-21](https://user-images.githubusercontent.com/82901720/137546503-a18a2a31-550b-4782-88b2-aed310f0bb4f.png)

* The aim of the project was to design a bot and implement self-balancing and line-following algorithms on it.
* The designing of the bot is done using solidworks and the self-balancing and line-following have been achieved using PID.

### Tech Stack
* ROS2 Foxy

### Tools
* Gazebo Version: 11.0
* RViz
* SolidWorks
* Autocad
* MS Paint

### File Structure
```
   ┣ 📂Bot_SolidWorks_Parts
   ┃ ┗ 📜WallE_Simulation_ROS2.zip               # Zip file containing all the SolidWorks files for our bot
   ┣ 📂assets
   ┃ ┗ 📜Project_Report.pdf                      # Project Report
   ┃ ┗ 📜WallE.gif                               # Gif of WallE bot performing self-balancing and line-following together
   ┣ 📂config
   ┃ ┗ 📜joint_names_Wall-E-urdf1.yaml           # Configuration file for joints of the bot
   ┣ 📂launch                                    # All launch files
   ┃ ┣ 📜emptyworld.launch.py
   ┃ ┣ 📜gazebo.launch.py
   ┃ ┣ 📜gzclient.launch.py
   ┃ ┣ 📜gzserver.launch.py
   ┃ ┣ 📜line_following.launch.py                # Launch file for bot for line-following algorithm
   ┃ ┣ 📜robot_state_publisher.launch.py         
   ┃ ┣ 📜self_balancing.launch.py                # Launch file for bot for self-balancing algorithm
   ┃ ┣ 📜walle.launch.py                         # Launch file for bot for self-balancing and line-following combined algorithm
   ┣ 📂meshes                                    # Meshes for different parts of bot
   ┃ ┣ 📜base_link.STL
   ┃ ┣ 📜chassis.STL
   ┃ ┣ 📜leftwheel.STL
   ┃ ┗ 📜rightwheel.STL
   ┣ 📂models
   ┃ ┗ 📜combinedworld.png                       # The png which decides the design of the world in Gazebo(for combined algorithm)
   ┃ ┣ 📜course.material                         # The file which links the world file to the png of the world
   ┃ ┣ 📜course2.material                        # The file which links the world file to the png of the world(for combined algorithm)
   ┃ ┗ 📜redline.png                             # The png which decides the design of the world in Gazebo(for combined algorithm)  
   ┃ ┗ 📜sra.png                                 # The png which decides the design of the world in Gazebo
   ┣ 📂rviz
   ┃ ┗ 📜urdf_config.rviz                        # The file for rviz configuration
   ┣ 📂src                                       # All cpp codes are stored here
   ┃ ┣ 📜line_following.cpp                      # The line-following algorithm
   ┃ ┣ 📜self_balancing.cpp                      # The self-balancing algorithm
   ┃ ┣ 📜wall_e.cpp                              # The self-balancing and line-following combined algorithm
   ┣ 📂urdf                                      # The sdf files for bot are stored here 
   ┃ ┣ 📜walle.csv
   ┃ ┣ 📜walle.urdf   
   ┃ ┣ 📜walle.sdf                               # The sdf file for self-balancing bot
   ┃ ┣ 📜walle2.sdf                              # The sdf file for line-following bot
   ┃ ┣ 📜walle4.sdf                              # The sdf file for line-following and self-balancing combined bot
   ┣ 📂worlds
   ┃ ┗ 📜sra.world                               # The line-following path
   ┃ ┗ 📜sra2.world                              # The line-following and self-balancing combined path
   ┣ 📜.gitignore
   ┣ 📜CMakeLists.txt                            # Contains all the information regarding the packages to be imported
   ┣ 📜LICENSE
   ┣ 📜README.md
   ┣ 📜build.sh                                  # The script executes the build and source commands from within the package
   ┣ 📜export.log
   ┣ 📜package.xml                               # Contains all the information regarding the dependencies to be imported
```  
    
## Getting Started

### Prerequisites

    1. ROS2 Foxy
    2. Gazebo Version: 11.0
    3. RViz 
    
* You can find the link for installtion of ROS2 Foxy in video format [here](https://youtu.be/fxRWY0j3p_U) and in text format [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). Note that the video is for installing ROS2 Foxy on Ubuntu whereas you can find the the instructions according to your 
own operating system in the official documentation provided in the second link above.
* The link to install Gazebo for Ubuntu is provided [here](http://gazebosim.org/tutorials?tut=install_ubuntu). You can find the instructions to install different versions of Gazebo in the link provided above but using Gazebo 11.0 is recommed for ROS2 Foxy.
* The link to install RViz on Ubuntu is provided [here](https://zoomadmin.com/HowToInstall/UbuntuPackage/rviz).

### Installation
1. Clone the repo
```
    git clone https://github.com/Aryaman22102002/Wall-e-simulation-ros2.git  
```

### Configuration
The colour of the bot can be varied by the changing the colour in the material tag of the visual element in the bot's sdf. The list of all colours and materials available for Gazebo are avialable [here](http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials).

## Usage
1. After cloning the repo, go into your ROS2 workspace and run the following commands:<br/>
```
colcon build       
```
```
. install/setup.bash
``` 

2. Now enter into the Wall-e-simulation-ros folder.

3. For launching the self-balancing bot, enter the following command:``` ros2 launch my_bot self_balancing.launch.py ```
 
4. For running the self-balancing code, enter the following command:``` ros2 run my_bot self_balancing ```
 
5. For launching the line-following bot, enter the following command:``` ros2 launch my_bot line_following.launch.py ```
  
6. For running the line-following code, enter the following command:``` ros2 run my_bot line_following ```

7. For launching the self-balancing plus line-following combined bot, enter the following command:``` ros2 launch my_bot walle.launch.py ```

8. For running the self-balancing plus line-following combined code, enter the following command:``` ros2 run my_bot wall_e ```

9. Everytime you make a change in the code/files, you will have to execute the build and source commands mentioned in point number 1. Alternatively, you can run the ``` build.sh ``` script which will execute both the commands mentioned in point number 1 from within the ROS 2 package. It will take the package name as input and execute those commands so that you don't have to go to the root of your workspace to build and source everytime you make a change. First create an executable for the script using the following command: <br>``` chmod +x build.sh ```

10. After creating an executable for the ``` build.sh ``` script as instructed in point number 9, to run the script, enter the following command: <br>``` . build.sh ```. 

## Theory and Approach

* The approach was to first design our bot using SolidWorks. Then we used ROS2 and Gazebo to simulate it. The main theory and concept used for self-balancing and line-following was Proportional Inetgral Derivative(PID).    
* A PID controller is one kind of device used to control different process variables like pressure, flow, temperature, and speed in industrial applications. Here, we first wrote the algorithm for self-balancing the bot. Then we wrote the algorithm for line-following. Then we tried to combine the two and again do the PID tuning.     
* For self-balancing the bot, we need to keep the bot a bit above our desired angle since the bot is very fast and wiill fall down once it's past the desired angle. The pitch angle is required for self-balancing which is obtained from the Inertial Measurement Unit(IMU sensor) plug-in.  
* For making the bot follow the line, we need to first set the frame of the rgb camera which then helps us to decide our desired position which in turn helps us to find the error(i.e by how much is the bot off from the desired position). We then apply PID and decide the angular velcoity which should be given to the bot and in which direction.
* For combining self-balancing and line-following algorithms, we need to ensure that the bot is balanced. Then only we can make it follow the line. At all times, the bot must give preference to balancing itself first and then following the line.  
* The following flow-chart describes the algorithm used for combining the self-balancing and line-following algorithms.

![combined algorithm(1)](https://user-images.githubusercontent.com/82901720/138567131-81471a10-4904-4cd4-8165-d7e93c8ba949.png)

## Results and Demo
The implementation of the self balancing and line following codes has been demonstarted in the following video.

https://user-images.githubusercontent.com/82901720/138333629-1ce269dd-8157-4214-a4d1-9e19adb14d49.mp4

<br>
    Please turn on the volume.
    
    
## Future Work
- [X] Combine self-balancing and line-following. 
- [ ] To implement the combined algorithm on hardware.
- [ ] Implement maze solving algorithms. 

## Troubleshooting 
* Sometimes the self-balancing code will behave differently than the previous run. Try to stop the execution of the code and launch the bot again and run the code.
* Always launch the bot and the codes from within the Wall-e-simulation-ros folder. Otherwise it may not the launch the world correctly.
* Also sometimes Gazebo might not launch inspite of running the launch commands. In such cases, try entering the command ``` killall gzserver ``` and then launching the bot again.

## Contributors
* [Aryaman Shardul](https://github.com/Aryaman22102002)<br/>
* [Marck Koothoor](https://github.com/marck3131)

## Acknowledgements and Resources
* [SRA Vjti](https://www.sravjti.in/) Eklavya 2021<br/>
* Special thanks to our mentors [Gautam Agrawal](https://github.com/gautam-dev-maker) and [Anushree Sabnis](https://github.com/MOLOCH-dev).<br/>
* Our [Project Report](https://github.com/Aryaman22102002/Wall-e-simulation-ros2/blob/main/assets/Project_Report.pdf)
* https://github.com/SRA-VJTI/Wall-E_v2.2
* My answer to why the meshes are not spawning in ROS2 : https://answers.gazebosim.org//question/26073/cannot-spawn-urdf-into-gazebo-using-ros2/ 
* https://navigation.ros.org/setup_guides/urdf/setup_urdf.html

## License
The [license](https://github.com/Aryaman22102002/Wall-e-simulation-ros/blob/main/LICENSE) used for this project.




  
      
 





 



