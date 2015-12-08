#!/usr/bin/env python
from irpos import *
from math import *
from hanoi import *

DISKS_NUMBER = 4 	# macro for number of disks
STARTING_POSE = [-0.0037306848274059677, -1.5662987527213679, 0.0724551380777767, -0.07714982552943606, 4.649474473338571, -1.57485125380838]


#TODO: zrobic makra z wielkosciami poszczegolnych krazkow

class Irp6Hanoi:
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	starting_rod = -1

	def move_to_starting_pose(self):
		self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
		#self.irpos.tfg_to_joint_position(0.087, 5.0)
		#self.irpos.move_rel_to_cartesian_pose_with_contact(20.0, Pose(Point(0.0, 0.0, 0.11), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,8.0),Vector3(0.0,0.0,0.0)))
		#self.irpos.move_to_joint_position(STARTING_POSE, 15.0)
				

	#def get_starting_rod(self):
	# TODO: zrobic funkcje, dla wykrywania czerwonego krazka
	
	#def shift_disk(self, src, dst, disk, disks_below):
	# TODO: funkcja dla przenoszenia zadanego dysku


if __name__ == '__main__':
#	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	obj = Hanoi(DISKS_NUMBER, 0)
	obj.solve_hanoi()
	obj.show_step_list()

	irp = Irp6Hanoi()
	irp.move_to_starting_pose()
