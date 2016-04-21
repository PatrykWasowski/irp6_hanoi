#!/usr/bin/env python
import rospy
import numpy
import sys 	# sys.exit()
from irpos import *
from hanoi import *
from hanoi_constants import *
from vis_builder import *
from std_msgs.msg import *
from threading import Lock
from visualization_msgs.msg import Marker

starting_rod = -1000
check_red = 0
received_rods = []
lock = threading.Lock()

class Irp6Hanoi:
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	visualisator = 0 
		
	rod_position = [[0 for x in range(5)] for x in range(5)] 
	rod_content = [[0 for x in range(DISKS_NUMBER)] for x in range(3)] 
	rod_position[LEFT] = [0, 0, 0, 0, 0, 0]	
	rod_position[CENTER] = [0, 0, 0, 0, 0, 0]	# position of IRp over left, center and right rod
	rod_position[RIGHT] = [0, 0, 0, 0, 0, 0]
	current_rod = 1
	current_disk = 0	
	carrying_disk = 0
	step_list = []		
	rviz_rods = []
	rod_content[LEFT] = []
	rod_content[CENTER] = []	# stacks that keep content of each rod 
	rod_content[RIGHT] = []
	move = [0, 0]
	is_above = 0
	rviz_enable = 0

	def callback_red(data):
		global starting_rod
		global check_red

		if(starting_rod == -1000 and check_red == 1):		
			if(data.data > 200):
				starting_rod = LEFT
			elif(data.data < -200):
				starting_rod = RIGHT
			else:
				starting_rod = CENTER

			#sub_red.shutdown()		# there is no point in further subscribing this topic
			print "Znaleziono czerwony krazek, jest on na pozycji " + str(starting_rod)
			check_red = 0

	def callback_rod(data):
		global received_rods
		global lock
		
		lock.acquire()
		received_rods = list(data.data)
		lock.release()

	sub_red = rospy.Subscriber('red_disk', Int16, callback_red)		# must be defined after callback definition
	sub_rod = rospy.Subscriber('rods', Int32MultiArray, callback_rod) 

	def set_step_list(self, steps):
		self.step_list = steps

	def get_starting_rod(self):
		return starting_rod

	def move_to_starting_pose(self):
		global check_red

		self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		self.irpos.tfg_to_joint_position(RED_DISK+0.009, 3.0)
		check_red = 1
		rospy.sleep(2.0)
		actual = self.irpos.get_cartesian_pose()
		self.visualisator = VisualisationBuilder(actual.position.z)
		model = Hanoi(DISKS_NUMBER, irp.get_starting_rod())
		model.solve_hanoi()
		model.show_step_list()
		self.set_step_list(model.get_step_list())


	def got_starting_rod(self):
		if(starting_rod == -1000):
			return 0
		else:
			if(starting_rod == LEFT):
				self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, -0.065, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			elif(starting_rod == RIGHT):
				self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.065, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			self.current_rod = starting_rod 				
			
			return 1

	def sort_points(self, l):
		pairs_nr = 0		
		pairs = []
		xs = []
		ret_l = []

		while(len(l) > 0):
			pairs.append( [ l.pop(), l.pop() ] )		
			xs.append( pairs[pairs_nr][0] )	
			pairs_nr += 1
		
		xs.sort()

		j = 0
		i = 0

		while(j < pairs_nr):
			ret_l.append(xs[j])
			while(i < pairs_nr):
				if(xs[j] == pairs[i][0]):
					ret_l.append(pairs[i][1])
					break
				else:
					i += 1  			
			j += 1
			i = 0

		return ret_l

	def compute_move(self, end):
		global received_rods
		global lock		

		lock.acquire()			
		src_center = [ received_rods.pop(), received_rods.pop() ]
		rods = received_rods
		lock.release()

		wanted_point = []
		rods_nr = len(rods) / 2	
		if(rods_nr == 0):
			print "Nie odnaleziono wiezy"
			sys.exit()
		else:
			rods = self.sort_points(rods)

		# choosing wanted point from received list of points of RodsDetection 

		if(end == LEFT):
			wanted_point = [ rods[0], rods[1] ]
		elif(end == RIGHT):
			wanted_point = [ rods[(rods_nr - 1) * 2], rods[(rods_nr - 1) * 2 + 1] ]
		else:
			wanted_point = [ rods[2], rods[3] ]
			
		# calculating move from found wanted point

		self.move[0] = Z / FX * (wanted_point[0] - src_center[0])
		self.move[1] = Z / FY * (wanted_point[1] - src_center[1])

		if(abs(self.move[0]) < 0.001 and abs(self.move[1]) < 0.001):
			self.is_above = 1

		print "Compute move finish " + str(self.move[0]) + " - " + str(self.move[1])
			
	def update_rviz(self, end):
		global received_rods
		global lock		

		lock.acquire()			
		src_center = [ received_rods.pop(), received_rods.pop() ]
		rods = received_rods
		lock.release()

		rods = self.sort_points(rods)

		actual = self.irpos.get_cartesian_pose()

		if(end == LEFT or end == CENTER):
			rod0 = LEFT
			position0 = [Z / FY * (rods[1] - src_center[1]) + actual.position.x, Z / FX * (rods[0] - src_center[0]) + actual.position.y]
			rod1 = CENTER
			position1 = [Z / FY * (rods[3] - src_center[1]) + actual.position.x, Z / FX * (rods[2] - src_center[0]) + actual.position.y]

			ratio = (rods[2] - rods[0]) / (rods[3] - rods[1]) 
			alpha = numpy.arctan(ratio)
			dx = numpy.cos(alpha) * ROD_DISTANCE			
			dy = numpy.sin(alpha) * ROD_DISTANCE
			if(end == LEFT):
				position1 = [ position0[0] - dx, position0[1] - dy]

			else:
				position0 = [position1[0] + dx, position1[1] + dy]

			rod2 = RIGHT
			position2 = [position1[0] - dx, position1[1] - dy]
		else:
			rod1 = CENTER
			position1 = [Z / FY * (rods[1] - src_center[1]) + actual.position.x, Z / FX * (rods[0] - src_center[0]) + actual.position.y]
			rod2 = RIGHT
			position2 = [Z / FY * (rods[3] - src_center[1]) + actual.position.x, Z / FX * (rods[2] - src_center[0]) + actual.position.y]
			
			if(rods[3] - rods[1] != 0):
				ratio = (rods[2] - rods[0]) / (rods[3] - rods[1]) 
			else:
				rospy.sleep(2.0)
				ratio = (rods[2] - rods[0]) / (rods[3] - rods[1]) 
							

			alpha = numpy.arctan(ratio)
			dx = numpy.cos(alpha) * ROD_DISTANCE			
			dy = numpy.sin(alpha) * ROD_DISTANCE

			position1 = [position2[0] + dx, position2[1] + dy]

			rod0 = LEFT
			position0 = [position1[0] + dx, position1[1] + dy]
		
		self.visualisator.set_rod_positions(rod0, position0, rod1, position1, rod2, position2)

		for i in range(3):
			self.visualisator.set_rod_content(i, self.rod_content[i])

		self.visualisator.draw_all()

	def move_to_rod(self, end):		
		print 'move_to_rod(' + str(end) + ")"
		
		if(self.rod_position[end] != [0, 0, 0, 0, 0, 0]):
			self.irpos.move_to_joint_position(self.rod_position[end], 4.0)	# when position of rod is known already		
			print "Pozycja juz jest znana, wiec sprobuje dawnego zapisu"
		else:
			diff = (end - self.current_rod) * ROD_DISTANCE
			self.irpos.move_rel_to_cartesian_pose(4.0,  Pose(Point(0.0, diff, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			print "Jeszcze nie znam pozycji, probuje na oko przesunac " + str(diff) 
		
		rospy.sleep(2.5)			

		if(self.carrying_disk):
			self.irpos.move_rel_to_cartesian_pose(1.0,  Pose(Point(-0.025, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			rospy.sleep(2.0)
			self.compute_move(end)
			while((abs(self.move[1]) - 0.025) > 0.005 or abs(self.move[0]) > 0.005):
				self.irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(-1*self.move[1], self.move[0], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
				self.irpos.move_rel_to_cartesian_pose(1.0,  Pose(Point(-0.025, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
				rospy.sleep(2.0)
				self.compute_move(end)
			self.irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(-1*self.move[1], self.move[0], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			self.current_rod = end
			 
	
		# first move has to be done, next moves depends on carrying disk
		else:
			self.compute_move(end)	
			self.irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(-1*self.move[1], self.move[0], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))	
			rospy.sleep(1.5)

			while (self.is_above == 0):					
				self.compute_move(end)			
				self.irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(-1*self.move[1], self.move[0], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
				print "Przesuwam " + str(self.move[1]) + " i " + str(self.move[0])								
				rospy.sleep(2.0) 				
			
			if(self.is_above):
				self.is_above = 0
				self.rod_position[end] = self.irpos.get_joint_position()
			
			if(self.rviz_enable == 1):		
				self.update_rviz(end)
			self.current_rod = end
			print "Dotarlem nad"	
		
	def get_disk_size(self, disk):
		if(disk == 1):
			return RED_DISK
		elif(disk == 2):
			return YELLOW_DISK
		elif(disk == 3):
			return GREEN_DISK
		elif(disk == 4):
			return BLUE_DISK

	def rod_name_from_nr(self, nr):
		if(nr == 0):
			return "LEFT"
		elif(nr == 1):
			return "CENTER"
		elif(nr == 2):
			return "RIGHT"

	def grabbing_height(self, disk, rod):
		hg = 0.12 - 0.014

		print "Len(rod_cont[" + str(rod) + "]) = " + str(len(self.rod_content[rod]))

		for i in range (0, len(self.rod_content[rod]) - 1):
			if(self.rod_content[rod][i] == 2):
				hg = hg - 0.0118
			else:
				hg = hg - 0.01		

		return hg

	def grab_disk(self, disk, disks_below, rod):
		print "Pobieram dysk " + str(disk)
		height = self.grabbing_height(disk, rod)
		print "Musze zjechac na dol o " + str(height)
		self.irpos.move_rel_to_cartesian_pose_with_contact(6.0, Pose(Point(0.0, 0.0, height), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		self.irpos.move_rel_to_cartesian_pose(0.5, Pose(Point(0.0, 0.0, -0.003), Quaternion(0.0, 0.0, 0.0, 1.0)))
		self.irpos.tfg_to_joint_position(self.get_disk_size(disk) - 0.004, 3.0) 
		print "Zlapalem dysk"

		#self.add_rviz_disks(disk, rod, 0, True)
		self.visualisator.move_disk(disk, rod)
		self.rod_content[rod].pop()

		self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.0, (-1)*height + 0.003), Quaternion(0.0, 0.0, 0.0, 1.0)))
					
	def leave_disk(self, disk, rod):
		print "Zrzucam dysk"
		self.irpos.move_rel_to_cartesian_pose_with_contact(3.0, Pose(Point(0.0, 0.0, 0.03), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		
		self.visualisator.put_disk(disk, rod)
		self.irpos.tfg_to_joint_position(MAX_DISK_WIDTH+DISK_STOCK, 3.0)
		print "Upuscilem dysk"
		self.irpos.move_rel_to_cartesian_pose(3.0, Pose(Point(0.0, 0.0, -0.03), Quaternion(0.0, 0.0, 0.0, 1.0)))		
	
		self.rod_content[rod].append(disk)
		self.rod_position[rod] = self.irpos.get_joint_position()		

		rospy.sleep(1.5)

		self.update_rviz(rod)

	def shift_disk(self, src, dst, disk, disks_below):
		print "Shift disk (" + str(src) + ", " + str(dst) + ", " + str(disk) + ", " + str(disks_below) +  ")"	

		rospy.sleep(2.)		# to be sure that new rod is found
		
		self.move_to_rod(src) 	
		
		self.grab_disk(disk, disks_below, src)
			
		self.carrying_disk = 1
		self.current_disk = disk		
		self.rod_position[src] = self.irpos.get_joint_position()
		
		self.move_to_rod(dst)

		self.leave_disk(disk, dst)
		self.carrying_disk = 0	
				

	def show_rod_content(self, nr):
		for i in range(len(self.rod_content[nr])):
			print self.rod_content[nr][i]

	#### RVIZ PART ####

	def add_rviz_rod(self, rod):
		marker = Marker()
		actual = self.irpos.get_cartesian_pose()

		marker.header.frame_id = "/pl_base"
		marker.ns = "basic_shapes"
		marker.id = 10 + rod
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.pose.position.x = actual.position.x
		marker.pose.position.y = actual.position.y
		marker.pose.position.z = actual.position.z - 0.108 + 0.009
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.095
		marker.color.r = 0.75
		marker.color.g = 0.70
		marker.color.b = 0.5
		marker.color.a = 1.0

		self.rviz_pub.publish(marker)

		# pink top of the rod
		marker.id = 30 + rod
		marker.type = marker.SPHERE
		marker.pose.position.z = marker.pose.position.z + 0.0475
		marker.scale.z = 0.01
		marker.color.r = 1.
		marker.color.g = 0.4
		marker.color.b = 0.4

		self.rviz_pub.publish(marker)

		if(rod == 1):
			marker.type = marker.CUBE
			marker.pose.position.z = actual.position.z - 0.143
			marker.scale.x = 0.079
			marker.scale.y = 0.23
			marker.scale.z = 0.014
			marker.color.r = 0.75
			marker.color.g = 0.70
			marker.color.b = 0.5
			marker.id = 20

			self.rviz_pub.publish(marker)

		marker.id = 30 + rod
		marker.type = marker.SPHERE
		marker.pose.position.z

	def add_rviz_disks(self, disk, rod, act, move):
		marker = Marker()
		if(move == True):
			marker.header.frame_id = "/pl_6"
			marker.frame_locked = True
			marker.pose.position.x = 0.
			marker.pose.position.y = 0.
			marker.pose.position.z = 0.27965
		else:
			marker.header.frame_id = "/pl_base"
			marker.frame_locked = False
			marker.pose.position.x = act.position.x
			marker.pose.position.y = act.position.y
			marker.pose.position.z = act.position.z - 0.143 + 0.012 + len(self.rod_content[rod]) * 0.01
			if(self.carrying_disk):
				marker.pose.position.z = marker.pose.position.z + 0.03 
					
		marker.ns = "basic_shapes"
		marker.id = disk
		marker.type = marker.CYLINDER
		marker.action = marker.ADD
		marker.pose.orientation.w = 1.0
		marker.scale.z = 0.01

		if(disk == 1):
			marker.color.r = 1.0
			marker.color.a = 1.0
			marker.scale.x = 0.027
		elif(disk == 2):
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.a = 1.0
			marker.scale.x = 0.031
		elif(disk == 3):
			marker.color.g = 1.0
			marker.color.a = 1.0
			marker.scale.x = 0.036
		elif(disk == 4):
			marker.color.b = 1.0
			marker.color.a = 1.0
			marker.scale.x = 0.04
		elif(disk == 5):
			marker.color.r = 1.0
			marker.color.g = 1.0			
			marker.color.b = 1.0
			marker.color.a = 1.0
			marker.scale.x = 0.027

		marker.scale.y = marker.scale.x

		self.rviz_pub.publish(marker)

	def delete_disk_rviz (self, disk):
		marker = Marker()
		marker.id = disk
		marker.action = marker.DELETE

		self.rviz_pub.publish(marker)

	def add_disk_to_rod(self, disk, rod):
		self.rod_content[rod].append(disk)
		
	def solve_hanoi(self):
		self.move_to_rod(2)
		self.move_to_rod(1)
		self.move_to_rod(0)
		self.rviz_enable = 1

		for i in range(4, 0, -1):
			self.rod_content[starting_rod].append(i)

		self.update_rviz(0)

		print "Wielkosc starting_rod " + str(len(self.rod_content[starting_rod]))
		print "Starting rod " + str(starting_rod)
		print str(len(self.step_list)) + " krokow do wykonania"

		for i in range(0, len(self.step_list)):
			self.shift_disk(self.step_list[i][0], self.step_list[i][1], self.step_list[i][2], self.step_list[i][3])
		print "FINITO"		


if __name__ == '__main__':

	irp = Irp6Hanoi()
	print "Utworzono obiekt Irp6Hanoi"
	irp.move_to_starting_pose()
	rospy.sleep(5.0)	
	
	print "Starting rod: " + str(irp.get_starting_rod()) 
	print "Koniec ustawiania"
	
	irp.solve_hanoi()
	# ROZWIAZYWANIE WIEZY HANOI	
