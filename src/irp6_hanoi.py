#!/usr/bin/env python
import rospy
from irpos import *
from hanoi import *
from std_msgs.msg import *

DISKS_NUMBER = 4 	# macro for number of disks
STARTING_POSE = [-0.0037306848274059677, -1.5662987527213679, 0.0724551380777767, -0.07714982552943606, 4.649474473338571, -1.57485125380838] 
RED_DISK = 0.067
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
	rod_position[0] = [0, 0, 0, 0, 0, 0]	
	rod_position[1] = [0, 0, 0, 0, 0, 0]	# position of IRp over left, center and right rod
	rod_position[2] = [0, 0, 0, 0, 0, 0]
	current_rod = 1
	carrying_disk = 0
	step_list = []		
	pub = rospy.Publisher('wanted_rod', String, queue_size = 5)  # pozniej uzywamy pub.publish("LEFT"/"RIGHT"/"CENTER")

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

		if(abs(data.data[0]) < 0.002 and abs(data.data[1]) < 0.002 and running == 0):
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

		self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		self.irpos.tfg_to_joint_position(RED_DISK+0.009, 3.0)
		#self.irpos.tfg_to_joint_position(0.087, 5.0)
		#self.irpos.move_rel_to_cartesian_pose_with_contact(20.0, Pose(Point(0.0, 0.0, 0.11), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		#self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		
		self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.0, -0.002), Quaternion(0.0, 0.0, 0.0, 1.0)))
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
			if(self.carrying_disk):			
				is_above = 1			
			print "Pozycja juz jest znana, wiec sprobuje dawnego zapisu"
		else:
			diff = (end - self.current_rod) * 0.07 
			self.irpos.move_rel_to_cartesian_pose(6.0,  Pose(Point(0.0, diff, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
			print "Jeszcze nie znam pozycji, probuje na oko przesunac " + str(diff) 
			rospy.sleep(1.)
					
		
		while (not is_above):
			if((not is_above) and (move != [0, 0])):
				running = 1				
				self.irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(-1*move[1], move[0], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
				print "Przesuwam " + str(move[1]) + " i " + str(move[0])								
				move = [0, 0]
				running = 0
				print "Czekam sekunde"
				rospy.sleep(1.)
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
		distance_to_disk = 0.122 - 0.014 - 0.01 * disks_below
		if(rod == 2 and disk == 1 and disks_below == 1):
			distance_to_disk = distance_to_disk - 0.004
		print "Musze zjechac na dol o " + str(distance_to_disk)
		self.irpos.move_rel_to_cartesian_pose_with_contact(6.0, Pose(Point(0.0, 0.0, distance_to_disk), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		self.irpos.move_rel_to_cartesian_pose(0.5, Pose(Point(0.0, 0.0, -0.003), Quaternion(0.0, 0.0, 0.0, 1.0)))
		self.irpos.tfg_to_joint_position(self.get_disk_size(disk) - 0.004, 3.0) 
		print "Zlapalem dysk"
		self.irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(0.0, 0.0, (-1)*distance_to_disk + 0.003), Quaternion(0.0, 0.0, 0.0, 1.0)))
					
	def leave_disk(self):
		print "Zrzucam dysk"
		self.irpos.move_rel_to_cartesian_pose_with_contact(3.0, Pose(Point(0.0, 0.0, 0.03), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		self.irpos.tfg_to_joint_position(MAX_DISK_WIDTH+DISK_STOCK, 3.0)
		print "Upuscilem dysk"
		self.irpos.move_rel_to_cartesian_pose(3.0, Pose(Point(0.0, 0.0, -0.03), Quaternion(0.0, 0.0, 0.0, 1.0)))

	def shift_disk(self, src, dst, disk, disks_below):
		print "Shift disk (" + str(src) + ", " + str(dst) + ", " + str(disk) + ", " + str(disks_below) +  ")"	

		self.pub.publish(self.rod_name_from_nr(src))
		rospy.sleep(2.)		# to be sure that new rod is found
		
		self.move_to_rod(src) 	
		
		self.pub.publish(self.rod_name_from_nr(dst))			# before robot moves to destination rod, the rod is already found 
		self.grab_disk(disk, disks_below, src)
		self.carrying_disk = 1
		self.rod_position[src] = self.irpos.get_joint_position()
		
		self.move_to_rod(dst)

		self.leave_disk()
		self.carrying_disk = 0	
		self.rod_position[dst] = self.irpos.get_joint_position()

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

	def solve_hanoi(self):
		self.move_to_rod(2)
		self.move_to_rod(1)
		

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
	
	irp.solve_hanoi()
	
	# ROZWIAZYWANIE WIEZY HANOI
		

