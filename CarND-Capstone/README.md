This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
#WORK FLOW#

##WHAT WE DO##
1. I selected native installation
   Install ROS Kinetic in Ubuntu 16.04
   Install Dataspeed DBW
   Install Udacity Simulator
2. Clone the project and copy to my github site
   Install the python dependencies
3. Run the ros server and the simulator
   in the directory CarND-Capstone/Ros,
   source devel/setup.sh
   roslaunch launch/styx.launch
   then run the simulator
4. overview the prject,and write the core code
5. real world testing

##CORE CODE
There are some main steps

###1.  Waypoint Updater Node
####Edit waypoint_updater.py 

(1) we get waypoints from /base_waypoints ,Msg Type is Lane

(2) get current position from /current_pose,Msg Type is PoseStamped

(3) create a KDTree used to get the closest waypoint from current position

(4) create a loop, publish the final waypoint

Now we can see the green waypoints from simulator


###2. DBW Node

####Edit twist_controller/dbw_node.py 

(1) get dbw_enabled param from /vehicle/dbw_enabled node,Msg Type is Bool.
init the Controller

(2) if dbw is enabled, call controller.control,and get throttle,brake,steer

####Edit twist_controller/twist_controller.py

(1) in control function,we care about these params:
	
* dbw_enabled
* goal_linear_v
* goal_angular_v
* brake_control
* current_linear_v
* dt

use pid controller get throttle
use yaw_controller get steer angle
use brake controller get the brake

###3. Traffic Light Detection Node

####Edit tl_detector/tl_detector.py
(1)there are two condition simulater light and real traffic light. 
we set a Bool param use_ground_truth to check the condition.

* In simulater light.
we get light position and light state from /vehicle/traffic_lights node
* In real light condition

we get light position from /vehicle/traffic_light node

**we get light state from /image_color node ,and called process_traffic_light function.**

(2)
we set a Int32 param last_ligth_wp,it is the traffic light's index position.
and publish it by /traffic_waypoint node.


####4.light_classification
This part is a core function.
we referenced the link https://github.com/alex-lechner/Traffic-Light-Classification

1. Datasets
    1. Extract images from a ROSbag file
roscore
rosbag play -l ./just_traffic_light.bag
here we create simulator/real two light conditions
    2. Data labeling
    3. Create a TFRecord file
2. Training
    1. Choosing a model
here we use SSD Inception V2
    2. Configure the .config file of the model
    4. Training the model
    5. Freezing the graph
3. Testing


###5. Full Waypoint Walkthrough

####Edit wapoint_updater.py

(1) combin the closest traffic light's position and state into the logic

(2) add a int param stopline_wp_idx,get from /traffic_waypoint node 
In the generate_lane function, if  stopline_wp_idx > 0 means the read traffic light
is in sight. if stopline_wp_idx < 0 mean no read light in sight.

##Project RUN
