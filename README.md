# EDLUT_BAXTER_DELAYS
Cerebellar-SNN control of a Baxter robot with sensorimotor time delays. The repository includes EDLUT simulator source code, the ROS packages to perform a closed-loop cerebellar-SNN control and configuration files needed. 


##  Requirements
* A computer with at least 8GB of RAM, a multicore CPU, an ENVIDIA GPU with CUDA support, and Linux (Ubuntu 15.10 or Ubuntu 16.04, required for ROS Kinetic) (Controller PC).
* A Baxter robot (it can be simulated using Gazebo, although its behaviour is completely different from the real one).
* A computer running Ubuntu 16.04 for monitoring purposes (Monitoring PC).
* A Raspberry Pi 3B+ (RPi). Only if WiFi connection is to be used between the robot and controller. In that case, the RPi operates as a WiFi robot-controller gateway.

## Installation
* Install and configure ROS (Kinetic distribution) and setup Baxter robot using the SDK Baxter guide: http://sdk.rethinkrobotics.com/wiki/Getting_Started (To do in the Controller PC, Monitoring PC and RPi).

* Install Gazebo (OPTIONAL): http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator . Notice that the dynamic model of the simulated version is completely different from the real robot, obtaining different results. 

* Download source code from repository https://github.com/EduardoRosLab/EDLUT_BAXTER_DELAYS and copy in home folder. 

* Install EDLUT simulator in the Controller PC (this step requires an NVIDIA GPU with CUDA support and CUDA installation) (To do only in the Controller PC):
	* Open a terminal and go to the folder EDLUT_source_code inside EDLUT_BAXTER_DELAYS repository.
	* $ chmod u+x configure
	* $ ./configure
	* $ make
	* $ sudo bash
	* $ make install

* Compile the ROS package (this step requires the installation of ROS and proper Baxter configuration)
	* In the Controller PC:
	* Open a terminal and go to the folder ros_ws (where the Baxter setup has been installed as described in http://sdk.rethinkrobotics.com/wiki/Getting_Started): $ cd ~/ros_ws
	* $ ./baxter.sh
	* $ cd ..
  	* Move to the ros package folder: $ cd EDLUT_BAXTER_DELAYS/rosPackages/ControllerPC
	* Compile: $ catkin_make
	* note: you can delete the EDLUT_BAXTER_DELAYS/rosPackages/MonitoringPC and EDLUT_BAXTER_DELAYS/rosPackages/RaspberryPi folders
	
	* In the Monitoring PC:
	* Open a terminal and go to the folder ros_ws (where the Baxter setup has been installed as described in http://sdk.rethinkrobotics.com/wiki/Getting_Started): $ cd ~/ros_ws
	* $ ./baxter.sh
	* $ cd ..
  	* Move to the ros package folder: $ cd EDLUT_BAXTER_DELAYS/rosPackages/MonitoringPC
	* Compile: $ catkin_make
	* note: you can delete the EDLUT_BAXTER_DELAYS/rosPackages/ControllerPC and EDLUT_BAXTER_DELAYS/rosPackages/RaspberryPi folders, as well as EDLUT_source_code and config_files
	

	* In the RPi:
	* Open a terminal and go to the folder ros_ws (where the Baxter setup has been installed as described in http://sdk.rethinkrobotics.com/wiki/Getting_Started): $ cd ~/ros_ws
	* $ ./baxter.sh
	* $ cd ..
  	* Move to the ros package folder: $ cd EDLUT_BAXTER_DELAYS/rosPackages/RaspberryPi
	* Compile: $ catkin_make
	* note: you can delete the EDLUT_BAXTER_DELAYS/rosPackages/ControllerPC and EDLUT_BAXTER_DELAYS/rosPackages/MonitoringPC folders, as well as EDLUT_source_code and config_files


* In the Controller PC: copy the content of config_files/neuron_models from EDLUT_BAXTER_DELAYS repository to ~/.ros/data . 



## Execution 
### Using real Baxter robot 
* In the Controller PC: Connect to Baxter robot following http://sdk.rethinkrobotics.com/wiki/Hello_Baxter 
  * Open a terminal: 
  * $ cd ros_ws/
  * $ ./baxter.sh 
  * Set Baxter publishing rate to 500 Hz: $ rostopic pub /robot/joint_state_publish_rate std_msgs/UInt16 500
  * $ cd ..
  * $ cd cd EDLUT_BAXTER_DELAYS/rosPackages/ControllerPC
  * $ source devel/setup.bash
  * Launch the desired experiment: 
    * Cerebellar-SNN torque control of a horizontal circle trajectory with artifically induced delays: 
	$ roslaunch edlut_ros artificial_delays_circle_trajectory_bio_delayYYms.launch 
	The cerebellar SNN accounts for a sensorimotor pathway delay of twice YYms (2x YYms), thus, each launch file varies the Kernel elegibility trace accordingly. 
    * Cerebellar-SNN torque control of an inclined circle trajetory with artificially induced delays: 
	$ roslaunch edlut_ros artificial_delays_inclined_circle_trajectory.launch 

    * PD Control of a horizontal circle trajectory with artirfically induced delays: 
	$ roslaunch edlut_ros PD_circle_delays.launch 

    * PD Control of an inclined circle trajectory with artifically induced delays: 
	$ roslaunch edlut_ros PD_inclined_circle_delays.launch 

    * Note: The induced artificial delays of the previous setups can be changed with ROS dynamic parameters (rqt_reconfigure ROS plugin). To do so: open a terminal, source ./baxter.sh script and run $ rosrun rqt_reconfigure rqt_reconfigure

    * Position Control of a horizontal circle trajectory with no delay: 
	$ roslaunch edlut_ros position_control_mode_circle_trajectory.launch
    
    * Position Control of an inclined circle trajectory with no delay: 
	$ roslaunch edlut_ros position_control_mode_inclined_circle_trajectory.launch
    

    * Cerebellar-SNN torque control over a WiFi connection of the horizontal circle trajectory: 
	$ roslaunch edlut_ros WiFi_circle_trajectory_learnt.launch

* To use the monitoring tools run the rqt_reconfigure ROS plugin. In the Monitoring PC:
  * Open a terminal: 
  * $ cd ros_ws/
  * $ ./baxter.sh 
  * $ rosrun rqt_reconfigure rqt_reconfigure (A new window will be displayed allowing the user to choose among several monitoring tools) 

