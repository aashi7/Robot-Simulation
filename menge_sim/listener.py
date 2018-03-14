#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData 
import math
import numpy as np 

import message_filters


flag_start = 1
prevPositionX = np.zeros((100))
prevPositionY = np.zeros((100))
linearVelocityX = np.zeros((100))
linearVelocityY = np.zeros((100))
prev_time = 0.0

class Map(object):

	"""
	The Map class stores an occupancy grid as a two dimensional numpy array.

	width - num of columns in occupancy grid
	height - num of rows in occupancy grid 
	resolution - width of each grid square in metre (10cm ~ 0.1m)
	origin_x - Position of the grid cell (0,0) in map coordinate system
	origin_y 
	grid   - numpy array with height rows and wisth columns

	x increases with increasing col number and y increases with inc row number  
	"""

	def __init__(self, origin_x, origin_y, resolution=0.1, width=50, height=50):

		self.origin_x  = origin_x 
		self.origin_y = origin_y 
		self.resolution = resolution
		self.width = width 
		self.height = height 
		self.grid = np.zeros((height, width))

	def to_message(self):

		grid_msg = OccupancyGrid()

		# Set up the header
		grid_msg.header.stamp = rospy.Time.now()
		grid_msg.header.frame_id = "map"

		# info is a nav_msgs/MapMetaData message
		grid_msg.info.resolution = self.resolution
		grid_msg.info.width = self.width 
		grid_msg.info.height = self.height 

		# Rotated maps are not supported.. quaternion represents no rotation 
		grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0), Quaternion(0, 0, 0, 1))

		# Flatten the numpy array into a list of integers from 0-100
		# This assumes that the grid entries are probabilities in the range 0-1
		flat_grid = self.grid.reshape((self.grid.size,))*100
		grid_msg.data = list(np.round(flat_grid))

		return grid_msg 

	def set_cell(self, x, y, val):

		## x,y - point in the map coordinte frame
		## val - value that should be assigned to grid cell that contains (x,y)

		## Throw the point that land outside the grid 
		## Process x,y to get grid_x, grid_y
		grid_x = 0
		grid_y = 0

		self.map.grid[grid_x, grid_y] = val 



def callback(robot_pose, ped_pose, laser_ends): 

	### Information not used yet 
	## I have the orientations of robot and pedestrians 
	# print ped_pose.poses[0].position.x   #works
	global prevPositionX     ## maximum 100 people at one time 
	global prevPositionY
	global linearVelocityX 
	global linearVelocityY 
	global prev_time
	global flag_start

	# print ped_pose
	# print len(ped_pose.poses)    ## length is coming to be 2 
	# print ped_pose.header.stamp.nsecs

	if (flag_start):	
		prev_time = rospy.get_time()
		for i in range(0, len(ped_pose.poses)):
			prevPositionX[i] = ped_pose.poses[i].position.x       
			prevPositionY[i] = ped_pose.poses[i].position.y	

		flag_start = 0	

		print prev_time
		print prevPositionX
		print prevPositionY
	
	else:
		curr_time = rospy.get_time()
		for i in range(0, len(ped_pose.poses)):
			linearVelocityX[i] = (ped_pose.poses[i].position.x - prevPositionX[i])/(curr_time - prev_time)
			linearVelocityY[i] = (ped_pose.poses[i].position.y - prevPositionY[i])/(curr_time - prev_time)
			prevPositionX[i]  = ped_pose.poses[i].position.x 
			prevPositionY[i]  = ped_pose.poses[i].position.y 

		prev_time = curr_time
		print linearVelocityX[0:len(ped_pose.poses)]
		print linearVelocityY[0:len(ped_pose.poses)]

	### Pedestrian positions I know, and robot's position too
	for ped in ped_pose.poses:
		## Distance of pedestrian from the robot
		distance = math.sqrt((ped.position.x - robot_pose.pose.position.x)**2 + (ped.position.y - robot_pose.pose.position.y)**2)
		# print (distance)

	### Find static obstacles
	for end in laser_ends.poses:
		distance = math.sqrt((end.position.x - robot_pose.pose.position.x)**2 + (end.position.y - robot_pose.pose.position.y)**2)
		# if (distance < 5):  ## some obstacle at the distance of less than 5 metres - wll capture both static and dynamic obstacle 
		# 	print (str(end.position.x) + " " + str(end.position.y))

	d_max = math.sqrt(5**2 + 2.5**2)   ## if distance of laser end/ crowd pose is less than this, I have to put them in local map around robot

	### I want to estimate the velocity of pedestrians 

	# prevPositionY = ped_pose.poses[0].position.y  




def listener():

	rospy.init_node('listener', anonymous=True)

	robot_pose = message_filters.Subscriber("pose", PoseStamped)    ## position of the robot 

	ped_pose = message_filters.Subscriber("crowd_pose", PoseArray)

	laser_ends =  message_filters.Subscriber("laser_end", PoseArray) 
	## If a laser_end lies within the local map - store it
	
	ts = message_filters.ApproximateTimeSynchronizer([robot_pose, ped_pose, laser_ends], queue_size=5, slop=0.1)
	ts.registerCallback(callback)

	rospy.spin()

if __name__ == '__main__':

	listener()


#### To subscribe to pose, laser_end at the same time, use time synchronizer 