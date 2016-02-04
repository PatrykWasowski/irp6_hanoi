#!/usr/bin/env python
import rospy
from irpos import *
from hanoi import *
from std_msgs.msg import *
from visualization_msgs.msg import Marker

DISKS_NUMBER = 4 	# macro for number of disks
STARTING_POSE = [-2.5289884940308865e-06, -1.5666245683871824, 0.07074496963619269, -0.07490522003488498, 4.712122248715821, -1.5710232858348003] 
RED_DISK = 0.0665
YELLOW_DISK = 0.071
GREEN_DISK = 0.077
BLUE_DISK = 0.080
MAX_DISK_WIDTH = 0.081	#  maximal disk width in metres
DISK_STOCK = 0.007	# zapas na szerokosc chwytaka przy podejmowaniu krazka odpowiedniego koloru
is_above = 0
move = [0, 0]
starting_rod = -1000
running = 0
check_red = 0
check_above = 0

class Irp6Hanoi:
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
		
	rod_position = [[0 for x in range(5)] for x in range(5)] 
	rod_content = [[0 for x in range(4)] for x in range(3)] 
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
	

	pub = rospy.Publisher('wanted_rod', String, queue_size = 5)  # pozniej uzywamy pub.publish("LEFT"/"RIGHT"/"CENTER")
	rviz_pub = rospy.Publisher("vis_rviz", Marker)

	def callback_red(data):
		global starting_rod
		global check_red
		
		if(starting_rod == -1000 and check_red == 1):		
			if(data.data > 200):
				starting_rod = LEFT
			elif(data.data < -300):
				starting_rod = RIGHT
			else:
				starting_rod = CENTER

			#sub_red.shutdown()		# there is no point in further subscribing this topic
			print "Znaleziono czerwony krazek, jest on na pozycji " + str(starting_rod)

	def callback_rod(data):
		global is_above
		global move
		global running
		global check_above

		if(abs(data.data[0]) < 0.001 and abs(data.data[1]) < 0.001 and running == 0):
			if(check_above == 1):			
				is_above = 1
				print "Jest nad"
				check_above = 0
		else:
			move = [data.data[0], data.data[1]]
			is_above = 0
	

	sub_red = rospy.Subscriber('red_disk', Int16, callback_red)
	sub_rod = rospy.Subscriber('rods', Float32MultiArray, callback_rod) 

	def set_step_list(self, steps):
		self.step_list = steps

	def move_to_starting_pose(self):
		global check_red

		#self.irpos.move_to_joint_position([ 0, -0.5 * math.pi, 0.0, 0, 1.5 * math.pi  , -0.5 * math.pi], 10.0)
		#self.irpos.move_rel_to_cartesian_pose_with_contact(20.0, Pose(Point(0.0, 0.0, 0.21), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		#self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.0, -0.12), Quaternion(0.0, 0.0, 0.0, 1.0)))

		self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		self.irpos.tfg_to_joint_position(RED_DISK+0.009, 3.0)
		#self.irpos.tfg_to_joint_position(0.087, 5.0)
		#self.irpos.move_rel_to_cartesian_pose_with_contact(20.0, Pose(Point(0.0, 0.0, 0.11), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		#self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		
		#self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.0, -0.002), Quaternion(0.0, 0.0, 0.0, 1.0)))
		check_red = 1
		rospy.sleep(2.)

	def got_starting_rod(self):
		global starting_rod		
		if(starting_rod == -1000):
			return 0
		else:
			if(starting_rod == LEFT):
				self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, -0.065, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			elif(starting_rod == RIGHT):
				self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.065, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			self.current_rod = starting_rod 				
			
						

			return 1
	
	def move_to_rod(self, end):		
		global is_above
		global move	
		global running	
		global check_above

		self.pub.publish(self.rod_name_from_nr(end))

		print 'move_to_rod(' + str(end) + ")"
		if(self.rod_position[end] != [0, 0, 0, 0, 0, 0]):
			self.irpos.move_to_joint_position(self.rod_position[end], 6.0)	# when position of rod is known already		
			print "Pozycja juz jest znana, wiec sprobuje dawnego zapisu"
			
		else:
			diff = (end - self.current_rod) * 0.07 
			self.irpos.move_rel_to_cartesian_pose(6.0,  Pose(Point(0.0, diff, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			print "Jeszcze nie znam pozycji, probuje na oko przesunac " + str(diff) 
					
		if(not self.carrying_disk):
			check_above = 1
		
		rospy.sleep(1.5)

		while (is_above == 0 and self.carrying_disk == 0):
			if((is_above == 0) and (move != [0, 0])):
				running = 1				
				self.irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(-1*move[1], move[0], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
				print "Przesuwam " + str(move[1]) + " i " + str(move[0])								
				move = [0, 0]
				running = 0
				print "Czekam sekunde"
				rospy.sleep(2.)
				check_above = 1
			
		
		if(is_above):
			is_above = 0
			self.rod_position[end] = self.irpos.get_joint_position()
			move = [0, 0]
			
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

	def grab_disk(self, disk, disks_below, rod):
		print "Pobieram dysk " + str(disk)
		height = self.grabbing_height(disk, rod)
		print "Musze zjechac na dol o " + str(height)
		self.irpos.move_rel_to_cartesian_pose_with_contact(6.0, Pose(Point(0.0, 0.0, height), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		self.irpos.move_rel_to_cartesian_pose(0.5, Pose(Point(0.0, 0.0, -0.003), Quaternion(0.0, 0.0, 0.0, 1.0)))
		self.irpos.tfg_to_joint_position(self.get_disk_size(disk) - 0.004, 3.0) 
		print "Zlapalem dysk"

		self.add_rviz_disks(disk, rod, 0, True)

		self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.0, (-1)*height + 0.003), Quaternion(0.0, 0.0, 0.0, 1.0)))
					
	def leave_disk(self):
		print "Zrzucam dysk"
		self.irpos.move_rel_to_cartesian_pose_with_contact(3.0, Pose(Point(0.0, 0.0, 0.03), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))

		actual = self.irpos.get_cartesian_pose()
		self.add_rviz_disks(self.current_disk, self.current_rod, actual, False)

		self.irpos.tfg_to_joint_position(MAX_DISK_WIDTH+DISK_STOCK, 3.0)
		print "Upuscilem dysk"
		self.irpos.move_rel_to_cartesian_pose(3.0, Pose(Point(0.0, 0.0, -0.03), Quaternion(0.0, 0.0, 0.0, 1.0)))

	def shift_disk(self, src, dst, disk, disks_below):
		print "Shift disk (" + str(src) + ", " + str(dst) + ", " + str(disk) + ", " + str(disks_below) +  ")"	

		self.pub.publish(self.rod_name_from_nr(src))
		rospy.sleep(2.)		# to be sure that new rod is found
		
		self.move_to_rod(src) 	
		
		self.add_rviz_rod(self.current_rod)

		self.pub.publish(self.rod_name_from_nr(dst))			# before robot moves to destination rod, the rod is already found 
		self.grab_disk(disk, disks_below, src)
		
		self.carrying_disk = 1
		self.current_disk = disk		
		self.rod_position[src] = self.irpos.get_joint_position()
		
		self.move_to_rod(dst)

		self.add_rviz_rod(self.current_rod)

		self.leave_disk()
		self.carrying_disk = 0	
		self.rod_position[dst] = self.irpos.get_joint_position()

		self.rod_content[dst].append(self.rod_content[src].pop())

		#for i in range(0, 3):
		#	self.show_rod_content(i)

	def grabbing_height(self, disk, rod):
		hg = 0.12 - 0.014

		for i in range (0, len(self.rod_content[rod]) - 1):
			if(self.rod_content[rod][i] == 2):
				hg = hg - 0.0118
			else:
				hg =hg - 0.01		

		return hg

	def show_rod_content(self, nr):
		for i in range(len(self.rod_content[nr])):
			print self.rod_content[nr][i]

	def solve_hanoi_test(self):
		global starting_rod
		
		self.move_to_rod(1)
		self.move_to_rod(2)
		self.move_to_rod(0)

		print "Zaczynam rozwiazywac Hanoi"
		self.pub.publish(self.rod_name_from_nr(starting_rod))
		self.move_to_rod(starting_rod)	
		rospy.sleep(2.)	
		self.grab_disk(1, 3)
		self.move_to_rod(1)
		self.leave_disk()

		self.move_to_rod(0)
		rospy.sleep(2.)
		self.grab_disk(2, 2)
		self.move_to_rod(2)
		self.leave_disk()
		
		self.move_to_rod(0)
		rospy.sleep(2.)
		self.grab_disk(3, 1)
		self.move_to_rod(1)
		self.leave_disk()

		self.move_to_rod(0)
		rospy.sleep(2.)
		self.grab_disk(4, 0)
		self.move_to_rod(2)
		self.leave_disk()

		
		#for i in range(0, len(self.step_list)):
		#	self.shift_disk(self.step_list[i][0], self.step_list[i][1], self.step_list[i][2], self.step_list[i][3])
		print "FINITO TEST"

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
		actual = self.irpos.get_cartesian_pose()
		self.add_rviz_disks(disk, rod, actual, False) 
		self.rod_content[rod].append(disk)
		
	def solve_hanoi(self):
		self.move_to_rod(2)
		self.add_rviz_rod(2)
		self.move_to_rod(1)
		self.add_rviz_rod(1)
		self.move_to_rod(0)
		self.add_rviz_rod(0)

		
		for i in range(4, 0, -1):
			self.add_disk_to_rod(i, starting_rod)

		print str(len(self.step_list)) + " krokow do wykonania"

		for i in range(0, len(self.step_list)):
			self.shift_disk(self.step_list[i][0], self.step_list[i][1], self.step_list[i][2], self.step_list[i][3])
		print "FINITO"		


if __name__ == '__main__':
#	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	obj = Hanoi(DISKS_NUMBER, 0)
	obj.solve_hanoi()
	obj.show_step_list()

	irp = Irp6Hanoi()
	print "Utworzono obiekt Irp6Hanoi"
	irp.set_step_list(obj.get_step_list())
	print "Zdobylem liste krokow"
	rospy.sleep(2.)
	irp.move_to_starting_pose()
	#print "Przesunalem sie do pozycji poczatkowej"
	#if(not irp.got_starting_rod()):
	#	print "Nie znalazlem czerwonego krazka"	
	#print "Magiczne liczby"
	#print irp.irpos.get_joint_position()
	irp.solve_hanoi()
	#actual = irp.irpos.get_cartesian_pose()
	#irp.add_rviz_disks(5, 0, actual, True)
	#irp.delete_disk_rviz(5)
	# ROZWIAZYWANIE WIEZY HANOI
		

