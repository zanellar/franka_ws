This is the ROS 1 workspace for interfacing with the Franka Emika Panda. CBF example controllers have been added. To run them on the Franka robot, follow the steps in this file.

Step 1: Change some paths
In src/franka_ros/franka_example_controllers/launch, under <!-- launch rosbag recorder -->, change the file path up to Rundata to the folder where you want to store your rosbag files. These contain all ROS messages sent during the experiment.

Step 2: Build and install
Open the Franka_WS folder in a Linux terminal and run:
catkin_make install
Source the script:
source ./install/setup.sh

The QP solver osqp depends on libabsl. For some reason, the .so files of this dependency are added to the developer folder, but not the install folder. (All cmake code in this project is incredibly botched, it's a miracle it compiles at all). As a workaround, one has to manually copy all libabsl.so files from Franka_WS/devel/lib to Franka_WS/install/lib.

Step 3: run examples on Franka robot:
After Step 2, in the same terminal:

roslaunch franka_example_controllers \
	[controller name].launch \
	 robot_ip:=172.16.0.2 \
	load_gripper:=false \
	robot:=panda \
	trajectory:=[string trajectory] \
	Kmax:=[double Kmax] \
	cbf_active:=[bool CBF active] \
	alpha:=[double alpha, or gamma] \
	rosbag

Notes:
- Controller name options: See franka_example_controllers_plugin.xml
- Trajectory options: See file names in franka_trajectory/src
- Many other parameters are available for the different controllers, see the respective launch files for those (recognizable by <arg name="param_name" ...)