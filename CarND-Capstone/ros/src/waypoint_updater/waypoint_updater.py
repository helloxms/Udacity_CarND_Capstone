#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int32,Float32,Float32MultiArray,Bool
from geometry_msgs.msg import PoseStamped,TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0

  
class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
	rospy.loginfo('waypoint updater init')
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
	self.stopline_wp_idx = -1
	self.brake_control = -1
	self.v = 0


        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
	rospy.Subscriber('/current_velocity',TwistStamped, self.velocity_cb)	


	#add a param to control braking,pub to dbw_node twist_controller
	self.brake_control_pub = rospy.Publisher('/brake_control',Float32,queue_size=1)


        #rospy.spin()
        self.loop()


    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
		
		if self.pose and self.base_waypoints:
			#Get closest waypoint
			#rospy.loginfo("get closest waypoint and get my pose")
			closest_waypoint_idx = self.get_closest_waypoint_idx()
			self.publish_waypoints(closest_waypoint_idx)
		rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
	y = self.pose.pose.position.y
	closest_idx = self.waypoint_tree.query([x,y],1)[1]
	closest_coord = self.waypoints_2d[closest_idx]
	prev_coord = self.waypoints_2d[closest_idx -1]

	# Equation for hyperplane through closest_coords
	cl_vect = np.array(closest_coord)
	prev_vect = np.array(prev_coord)
	pos_vect = np.array([x,y])

	val = np.dot(cl_vect-prev_vect, pos_vect -cl_vect)
	if val >0:
		closest_idx = (closest_idx + 1)%len(self.waypoints_2d)
	return closest_idx

    def publish_waypoints(self, closest_idx):
	#lane = Lane()
	#lane.header = self.base_waypoints.header
	#lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]
	final_lane = self.generate_lane()
	self.final_waypoints_pub.publish(final_lane)
	self.brake_control_pub.publish(self.brake_control)

    def generate_lane(self):
	lane = Lane()
	closest_idx = self.get_closest_waypoint_idx()
	farthest_idx = closest_idx + LOOKAHEAD_WPS
	N = len(self.base_waypoints.waypoints)
	#rospy.logwarn("waypoints len=%d", N)
	recycle = False
	recycle_len = 0
	if farthest_idx > N-1:
		farthest_idx = N-1
		recycle = True
		recycle_len = closest_idx + LOOKAHEAD_WPS-N
	base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
	if recycle == True:
		recycle_points = self.base_waypoints.waypoints[0:recycle_len]
		for i in recycle_points:
			base_waypoints.append(i)
	
	#rospy.logwarn("closest idx = %d", closest_idx)
	#rospy.logwarn("farthest idx = %d", farthest_idx)
	#rospy.logwarn("stopline idx = %d", self.stopline_wp_idx) 
	self.brake_control = -1
	if self.stopline_wp_idx == -1 :
		rospy.loginfo("no read light,keep running")
		lane.waypoints = base_waypoints
	elif (self.stopline_wp_idx >= farthest_idx):

		rospy.loginfo("get read light,but is far away")
		lane.waypoints = base_waypoints
		
	else:
		rospy.loginfo("get read light,need stop")
		lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
		#self.brake_control = 1
		

	return lane


    #create a new waypoints to slowdown and stop the car
    def decelerate_waypoints(self, waypoints, closest_idx):

	temp = []
	stop_pos = self.base_waypoints.waypoints[self.stopline_wp_idx]
	stop_distance = self.distance(closest_idx, self.stopline_wp_idx)


	if stop_distance < 6:
		self.brake_control = 1
	else:
		if self.v >0:
			t = (2*stop_distance)/(self.v)
			self.brake_control = min(3, self.v/max(0.01, t))
	for i,wp in enumerate(waypoints):
		p = Waypoint() 
		p.pose = wp.pose
                #here is a extra logic
		#here waypoints's length is farthest_idx - closest_idx = 200 LOOKAHEAD_WPS
                #stop_idx is the cur waypoints's stop index, the velocity should be 0 after is index

		stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
		#if stop_idx <10:
		#	self.brake_control = 1
		vel = 0.
		if i <= stop_idx and stop_idx < len(waypoints):
			dist = self.distance(closest_idx+i, closest_idx+stop_idx)
			vel = math.sqrt(2 * MAX_DECEL * dist)
		else:
			vel = 0.
	

		if vel < 1.:
			vel = 0.
	
		
		p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
		#self.set_wapoint_velocity(waypoints,i,cur_v)

		temp.append(p)
	return temp


    def pose_cb(self, msg):
        # TODO: Implement
	self.pose = msg
	#rospy.loginfo("pose callback called")
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	rospy.logwarn("waypoints callback called ")
	self.base_waypoints = waypoints
	if not self.waypoints_2d:
		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
		self.waypoint_tree = KDTree(self.waypoints_2d)
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
	#rospy.loginfo("traffic light callback called msg.data=%d", msg.data)
        # TODO: Callback for /traffic_waypoint message. Implement
	self.stopline_wp_idx = msg.data
     

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def velocity_cb(self, msg):
	self.v = msg.twist.linear.x
	#rospy.loginfo("cur velocity v=%f",self.v)

	#there is a sycle in waypoints distance 
    def distance(self, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	N = len(self.base_waypoints.waypoints)
	if wp2 >= wp1:
		for i in range(wp1,wp2):
			dist += dl(self.base_waypoints.waypoints[i].pose.pose.position,self.base_waypoints.waypoints[i+1].pose.pose.position)
	else:
		for i in range(wp1, N-1):
			dist += dl(self.base_waypoints.waypoints[i].pose.pose.position,self.base_waypoints.waypoints[i+1].pose.pose.position)
		for i in range(wp2):
			dist += dl(self.base_waypoints.waypoints[i].pose.pose.position,self.base_waypoints.waypoints[i+1].pose.pose.position)

        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
