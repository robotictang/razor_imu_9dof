Official ROS Documentation
--------------------------
A standard ROS-style version of this documentation can be found on the ROS wiki at:

http://wiki.ros.org/razor_imu_9dof

Install and Configure ROS Package
---------------------------------
1) Install dependencies:

	$ sudo apt-get install python-visual

2) Download code:

	$ cd ~/catkin_workspace/src
	$ git clone https://github.com/robotictang/razor_imu_9dof.git
	$ cd ~/catkin_workspace
	$ catkin_make

3) Edit launch/razor.launch to use correct USB port:

	<param name="device" type="string" value="/dev/ttyUSB0" />


Install Arduino firmware
-------------------------
1) Open src/Razor_AHRS/Razor_AHRS.ino in Arduino IDE

2) Select your hardware here by uncommenting the right line in src/Razor_AHRS/Razor_AHRS.ino, e.g.

<pre>
// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
</pre>

3) Upload Arduino sketch


Launch
------
	
	$ roslaunch razor_imu_9dof razor.launch

