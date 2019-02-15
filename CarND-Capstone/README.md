
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
