#!/usr/bin/env python
#from irpos import *
#from math import *
from hanoi_constants import *

class Hanoi:
	rods = []	
	left_rod = []
	center_rod = []
	right_rod = []
	rods.append(left_rod)
	rods.append(center_rod)
	rods.append(right_rod)
	starting_rod = -1
	disks_number = 0
	step_list = []

	def __init__(self, nr, rod):
		self.disks_number = nr 
		self.starting_rod = rod 	# rod that contains disks on start
		self.buffering_rod = (rod + 1) % 3	
		self.target_rod = (rod + 2) % 3

		if self.starting_rod == LEFT:
			for i in range (self.disks_number, 0, -1):			
				self.left_rod.append(i)
		elif self.starting_rod == CENTER:
			for i in range (self.disks_number, 0, -1):		
				self.center_rod.append(i)
		else:
			for i in range (self.disks_number, 0, -1):			
				self.right_rod.append(i)

	def show_rods (self):
		for i in range (self.disks_number, 0, -1):
			if len(self.left_rod) >= i:
				napis = '  ' + `self.left_rod[i-1]`
			else:
				napis = '  |'
			napis += '   '

			if len(self.center_rod) >= i:
				napis += `self.center_rod[i-1]`
			else:
				napis += '|'
			napis += '   '

			if len(self.right_rod) >= i:
				napis += `self.right_rod[i-1]`
			else:
				napis += '|'
			print napis
		print ' -----------'
	
	def shift_disk (self, source_rod, target_rod):
		disk = self.rods[source_rod].pop()		
		self.rods[target_rod].append(disk)
		self.show_rods()
		return disk

	def solve (self, src, dst, buff, n):
		if n == 1:
			disk = self.shift_disk(src, dst)
			number_of_disks_below = len(self.rods[src])
			self.step_list.append((src, dst, disk, number_of_disks_below)) 
		else:
			self.solve(src, buff, dst, n-1)
			self.solve(src, dst, buff,1)
			self.solve(buff, dst, src, n-1)

	def solve_hanoi(self):
		self.solve(self.starting_rod, self.target_rod, self.buffering_rod, self.disks_number)
		
	def show_step_list(self):
		for i in range (0, len(self.step_list)):
			print str(self.step_list[i][0]) + ' -> ' + str(self.step_list[i][1]) + '(' + str(self.step_list[i][2]) + ', ' + str(self.step_list[i][3]) + ')'  
	
	def get_step_list(self):
		return self.step_list

if __name__ == '__main__':
	obj = Hanoi(4, 0)
	obj.show_rods()
	obj.solve(obj.starting_rod, obj.target_rod, obj.buffering_rod, obj.disks_number)
	obj.show_step_list()
