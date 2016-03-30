#!/usr/bin/env python
import rospy
import numpy
from hanoi_constants import *
from std_msgs.msg import *
from visualization_msgs.msg import Marker

class VisualisationBuilder:
	rviz_pub = rospy.Publisher('vis_rviz', Marker)
	rod_position = [[0 for x in range(2)] for x in range(3)] 
	rod_position[LEFT] = [0, 0]	
	rod_position[CENTER] = [0, 0]	# position of IRp over left, center and right rod
	rod_position[RIGHT] = [0, 0]
	rod_content = [[0 for x in range(DISKS_NUMBER)] for x in range(3)]
	position_z = 0
	
	def __init__(self, z):
		self.position_z = z + DISTANCE_ROD # setting value of z

	def set_rod_position(self, rod, position):
		self.rod_position[rod] = position

	def set_invisible_rod_position(self, inv_rod):
		if(inv_rod == LEFT):
			x0 = 2 * self.rod_position[CENTER][0] - self.rod_position[RIGHT][0]
			y0 = 2 * self.rod_position[CENTER][1] - self.rod_position[RIGHT][1]
			self.rod_position[LEFT] = [x0, y0]
		elif(inv_rod == RIGHT):
			x0 = 2 * self.rod_position[CENTER][0] - self.rod_position[LEFT][0]
			y0 = 2 * self.rod_position[CENTER][1] - self.rod_position[LEFT][1]
			self.rod_position[RIGHT] = [x0, y0]

	def set_rod_positions(self, rod0, position0, rod1, position1, rod2, position2):
		self.set_rod_position(rod1, position1)
		if(rod0 != -1 and rod2 != -1):
			self.set_rod_position(rod0, position0)
			self.set_rod_position(rod2, position2)
		elif(rod0 == -1):
			self.set_rod_position(rod2, position2)
			self.set_invisible_rod_position(LEFT)
		elif(rod2 == -1):
			self.set_rod_position(rod0, position0)
			self.set_invisible_rod_position(RIGHT)

	def set_rod_content(self, rod, content):
		if(rod != -1):		
			self.rod_content[rod] = content

	def draw_rod(self, rod):
		marker = Marker()
		
		marker.header.frame_id = "/pl_base"
		marker.ns = "basic_shapes"
		marker.id = 10 + rod
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.pose.position.x = self.rod_position[rod][0]
		marker.pose.position.y = self.rod_position[rod][1]
		marker.pose.position.z = self.position_z
		marker.pose.orientation.w = 1.0
		marker.scale.x = ROD_SCALE_XYZ[0]
		marker.scale.y = ROD_SCALE_XYZ[1]
		marker.scale.z = ROD_SCALE_XYZ[2]
		marker.color.r = ROD_COLOR_RGB[0]
		marker.color.g = ROD_COLOR_RGB[1]
		marker.color.b = ROD_COLOR_RGB[2]
		marker.color.a = 1.0

		self.rviz_pub.publish(marker)

		# pink top of the rod
		marker.id = 30 + rod
		marker.type = marker.SPHERE
		marker.pose.position.z = marker.pose.position.z + 0.0475
		marker.scale.z = TOP_SCALE_Z
		marker.color.r = TOP_COLOR_RGB[0]
		marker.color.g = TOP_COLOR_RGB[1]
		marker.color.b = TOP_COLOR_RGB[2]

		self.rviz_pub.publish(marker)

	def draw_base(self):
		marker = Marker()
		ratio = (self.rod_position[CENTER][1] - self.rod_position[LEFT][1]) / (self.rod_position[CENTER][0] - self.rod_position[LEFT][0]) 
		alpha = numpy.arctan(ratio)

		marker.header.frame_id = "/pl_base"
		marker.ns = "basic_shapes"
		marker.id = 10
		marker.type = marker.CUBE
		marker.action = marker.ADD
		marker.pose.position.x = self.rod_position[CENTER][0]
		marker.pose.position.y = self.rod_position[CENTER][1]
		marker.pose.position.z = self.position_z + DIFF_DISTANCE_BASE
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0		
		marker.pose.orientation.z = numpy.sin(alpha / 2)
		marker.pose.orientation.w = numpy.cos(alpha / 2) 		
		marker.scale.x = BASE_SCALE_XYZ[0]
		marker.scale.y = BASE_SCALE_XYZ[1]
		marker.scale.z = BASE_SCALE_XYZ[2]
		marker.color.r = ROD_COLOR_RGB[0]
		marker.color.g = ROD_COLOR_RGB[1]
		marker.color.b = ROD_COLOR_RGB[2]
		marker.color.a = 1.0

		self.rviz_pub.publish(marker)

	def draw_disk(self, disk, rod):
		marker = Marker()
		print "DRAW DISK " + str(rod)
		
		marker.header.frame_id = "/pl_base"
		marker.frame_locked = False
		marker.pose.position.x = self.rod_position[rod][0]
		marker.pose.position.y = self.rod_position[rod][1]
		marker.pose.position.z = self.position_z + DIFF_DISTANCE_BASE + 0.012 + len(self.rod_content[rod]) * 0.01
					
		marker.ns = "basic_shapes"
		marker.id = disk
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.pose.orientation.w = 1.0
		marker.scale.x = DISK_SIZE[disk - 1]
		marker.scale.y = DISK_SIZE[disk - 1]
		marker.scale.z = 0.01
		marker.color.r = DISK_COLOR[disk - 1][0]
		marker.color.g = DISK_COLOR[disk - 1][1]
		marker.color.b = DISK_COLOR[disk - 1][2]
		marker.color.a = 1.0

		print "FINISH DRAW"
	
		self.rviz_pub.publish(marker)

	def move_disk(self, disk):
		marker = Marker()

		marker.id = disk		
		marker.action = marker.ADD
		marker.header.frame_id = "/pl_6"
		marker.frame_locked = True
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.27965
		
		self.rviz_pub.publish(marker)

	def put_disk(self, disk, rod):
		marker = Marker()
		
		marker.id = disk
		marker.action = marker.ADD
		marker.header.frame_id = "/pl_base"
		marker.frame_locked = False
		marker.pose.position.x = self.rod_position[rod][0]
		marker.pose.position.y = self.rod_position[rod][1]
		marker.pose.position.z = self.position_z + DIFF_DISTANCE_BASE + 0.012 + len(self.rod_content[rod]) * 0.01

		self.rviz_pub.publish(marker)

	def draw_all(self):
		self.draw_base()
		for i in range(3):
			self.draw_rod(i)
	
		for i in range(3):
			rod_content_copy = self.rod_content[i]
			for j in range(len(rod_content_copy)):
				print "draw_disk - " + str(j)
				self.draw_disk(rod_content_copy.pop(), i)

	def update(self, rod_data0, rod_data1, rod_data2):
		self.set_all(rod_data0, rod_data1, rod_data2)
		self.draw_all()
