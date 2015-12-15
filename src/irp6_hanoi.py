#!/usr/bin/env python
from irpos import *
from hanoi import *
from std_msgs.msg import *

DISKS_NUMBER = 4 	# macro for number of disks
STARTING_POSE = [-0.0037306848274059677, -1.5662987527213679, 0.0724551380777767, -0.07714982552943606, 4.649474473338571, -1.57485125380838] 
RED_DISK = 0.067
YELLOW_DISK = 0.072
GREEN_DISK = 0.078
BLUE_DISK = 0.082
MAX_DISK_WIDTH = 0.082	#  maximal disk width in metres
DISK_STOCK = 0.005	# zapas na szerokość chwytaka przy podejmowaniu krążka odpowiedniego koloru


class Irp6Hanoi:
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	starting_rod = -1000
	pub = rospy.Publisher('wanted_rod', String, queue_size = 5)  # pozniej uzywamy pub.publish("LEFT"/"RIGHT"/"CENTER")
	sub_red = rospy.Subscriber('red_disk', Int16, callback_red)
	sub_rod = rospy.Subscriber('rods', Int16MultiArray, callback_rod) 
	rod_position[LEFT] = [0, 0, 0, 0, 0, 0]	
	rod_position[CENTER] = [0, 0, 0, 0, 0, 0]	# position of IRp over left, center and right rod
	rod_position[RIGHT] = [0, 0, 0, 0, 0, 0]
	current_rod = -1
	move = [0, 0]
	is_above = 0
	step_list = []	

	def set_step_list(self, steps):
		step_list = steps

	def move_to_starting_pose(self):
		self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		self.irpos.tfg_to_joint_position(MAX_DISK_WIDTH+DISK_STOCK, 3.0)
		#self.irpos.tfg_to_joint_position(0.087, 5.0)
		#self.irpos.move_rel_to_cartesian_pose_with_contact(20.0, Pose(Point(0.0, 0.0, 0.11), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		#self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
				
	def callback_red(data):
		if(data.data > 200):
			self.starting_rod = LEFT
		elif(data.data < -300):
			self.starting_rod = RIGHT
		else:
			self.starting_rod = CENTER

		self.sub_red.shutdown()		# there is no point in further subscribing this topic

	def callback_rod(data):
		if(data.data[0] < 3 && data.data[1] < 3):
			is_above = 1
		else:
			move = [data.data[0], data.data[1]]	

	def got_starting_rod(self):
		if(starting_rod == -1000):
			return 0
		else:
			if(self.starting_rod == LEFT):
				self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, -0.06, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			elif(self.starting_rod == RIGHT):
				self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.06, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			self.current_rod = self.starting_rod 				
			
			return 1
	
	def move_to_rod(self, end):
		if(self.rod_position[end] != [0, 0, 0, 0, 0, 0]):
			self.irpos.move_to_joint_position(rod_position[end])	# when position of rod is known already
			self.current_rod = end
			return 1
		else:
			if(self.is_above):
				self.is_above = 0
				self.rod_position[end] = self.irpos.get_joint_position()
				self.move = [0, 0]
				self.current_rod = end
				return 1
			elif(!self.is_above && move != [0, 0]):
				self.irpos.move_rel_to_cartesian_pose(3.0, Pose(Point(0.0, self.move(0) / 43.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
				self.move = [0, 0]
				return 0

	def get_disk_size(self, disk):
		if(disk == 1):
			return RED_DISK
		elif(disk == 2):
			return YELLOW_DISK
		elif(disk == 3):
			return GREEN_DISK
		elif(disk == 4):
			return BLUE_DISK

	def grab_disk(self, disk, disks_below):
		distance_to_disk = 12 - 1.4 - 1.0 * disks_below
		self.irpos.move_rel_to_joint_position_with_contact(5.0, Pose(Point(0.0, 0.0, distance_to_disk), Quaternion(0.0, 0.0, 0.0, 1.0)))
		self.irpos.tfg_to_joint_position(self.get_disk_size(disk), 3.0)
		self.irpos.move_rel_to_joint_position(5.0, Pose(Point(0.0, 0.0, (-1.0)*distance_to_disk), Quaternion(0.0, 0.0, 0.0, 1.0)))
					
	def leave_disk(self):
		self.irpos.move_rel_to_joint_position_with_contact(5.0, Pose(Point(0.0, 0.0, 2.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
		self.irpos.tfg_to_joint_position(MAX_DISK_WIDTH+DISK_STOCK, 3.0)
		self.irpos.move_rel_to_joint_position_with_contact(5.0, Pose(Point(0.0, 0.0, -2.0), Quaternion(0.0, 0.0, 0.0, 1.0)))

	def shift_disk(self, src, dst, disk, disks_below):
		ready = 0		
		while(!ready)
			ready = self.move_to_rod(src) 

		self.grab_disk(disk, disks_below)
		
		ready = 0
		while(!ready)
			ready = self.move_to_rod(dst)

		self.leave_disk()	

	def solve_hanoi(self):
		for i in range(0, len(self.step_list)):
			self.shift_disk(self.step_list[i][0], self.step_list[i][1], self.step_list[i][2], self.step_list[i][3])

if __name__ == '__main__':
#	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	obj = Hanoi(DISKS_NUMBER, 0)
	obj.solve_hanoi()
	obj.show_step_list()

	irp = Irp6Hanoi()
	irp.set_step_list(obj.get_step_list())
	irp.move_to_starting_pose()
	while(!got_starting_rod()):
		print "Starting rod not detected\n"
	
	#ROZPOCZNIJ ROZWIAZYWANIE WIEZY HANOI
		

