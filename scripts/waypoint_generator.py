#! /usr/bin/env python3.3 

import time
from pymorse import Morse

if __name__ == '__main__':
	with Morse() as morse:
		print("Press q to quit, r to record a waypoint, n to insert a new sequence of waypoints, and c to make another file of waypoints")
		esc = False
		index = 0
		file_name = "./Waypoints/wp" + str(index) + ".txt"	
		wp_file = open(file_name, "w")	

		while not esc:
			c = input()
			
			if c == "r":
				morse_command = ['human' + str(0) + '.pose' + str(0), 'get_local_data']
				pose = morse.rpc(morse_command[0], morse_command[1])
				pose = [round(pose['x'],2), round(pose['y'],2)]
				wp_file.write(str(pose[0]) + " ")
				wp_file.write(str(pose[1]) + " ")
			elif c == "n":
				wp_file.write("\n")
			elif c == "q":
				esc = True
			elif c == "c":
				wp_file.close()
				time.sleep(1)
				index = index + 1
				file_name = "./Waypoints/wp" + str(index) + ".txt"
				wp_file = open(file_name, "w")
				
		wp_file.close()
