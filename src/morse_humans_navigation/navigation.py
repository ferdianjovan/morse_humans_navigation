#! /usr/bin/env python3.3 

import rospy
import os
import os.path
import pymorse
import time
import math
import yaml
import random
import threading

class Human:
    def __init__(self, wp_seqs, wp_dict, movement_plan, morse, human_id, total_human):
        self.human_id = human_id
        self.wp_dict = wp_dict
        self.movement_plan = movement_plan
        self.morse = morse
        self.total_human = total_human
        self.waypoint_sequences = wp_seqs
        self.last_10pos = []
        self.range_max = 0.6
        self.waypoint_speed = 1 
        self.waypoint_tolerance = 0.5
        worker = threading.Thread(target=self.move_planning)
        worker.setDaemon(True)
        worker.start()

    def move_planning(self):
        while True:
            waypoints = []
            pose = self.get_pose(self.human_id)
    
            # finding which waypoint human currently is
            wp_name = ""
            for i,j in self.wp_dict.items():
                if self.is_near(pose[0], j[0], pose[1], j[1], 1):
                         wp_name = i
                         break
    
            # plan the next waypoint
            print("Human" + str(self.human_id) + " is in " + wp_name)
            #next_wp_name = self.movement_plan[wp_name][randint(0,
            #    len(self.movement_plan[wp_name])-1)]
            max_rand = 0
            for val in self.movement_plan[wp_name].values():
                max_rand += val

            low_thres = 0
            rand = random.random() * max_rand
            for keys, vals in self.movement_plan[wp_name].items():
                if low_thres <= rand and rand < vals + low_thres:
                    next_wp_name = keys
                    break
                else:
                    low_thres += vals

            next_wp = self.wp_dict[next_wp_name]
    
            if next_wp_name != wp_name:
                # get the waypoint sequences   
                print("Human" + str(self.human_id) + " will be going to " + next_wp_name) 
                wp_threshold = 0.4
                next_wp_threshold = 0.4
                while waypoints == []:
                    for i in self.waypoint_sequences:
                        if self.is_near(next_wp[0],i[len(i)-1][0],next_wp[1],i[len(i)-1][1],next_wp_threshold):
                            if self.is_near(pose[0], i[0][0], pose[1], i[0][1], wp_threshold):
                                waypoints = i
                    wp_threshold += 0.1
                    next_wp_threshold += 0.1
                    
                self.move(waypoints)
    
            time.sleep(30)
    
    def get_pose(self,human_id):
        morse_command = ['human' + str(human_id) + '.pose' + str(human_id),
        'get_local_data']
        pose = self.morse.rpc(morse_command[0], morse_command[1])
        return [round(pose['x'],2),round(pose['y'],2)]

    def is_near(self,x0, x1, y0, y1, threshold):
        if abs(x0 - x1)<=threshold and abs(y0 - y1)<=threshold:
            near = True
        else: 
            near = False
        return near
    
    def stuck(self):
        really_stuck = True 
    
        for i in range(0,len(self.last_10pos) - 1):
            if abs(self.last_10pos[i][0] - self.last_10pos[i+1][0]) > 0.1 or \
            abs(self.last_10pos[i][1] - self.last_10pos[i+1][1]) > 0.1: 
                really_stuck = False
                break
    
        return really_stuck

    def obstacle_avoidance(self):
        obr = False; obl = False; obml = False; obmr = False
        morse_command = ['human' + str(self.human_id) + '.laser' + str(self.human_id), 'get_local_data']
        range_list = self.morse.rpc(morse_command[0], morse_command[1])['range_list']
        morse_command = 'human' + str(self.human_id) + '.motion' + str(self.human_id)

        for i in range(2, len(range_list) - 2):
            if range_list[i] < self.range_max and i <= len(range_list) / 2:
                obr = True
            elif range_list[i] < self.range_max and i > len(range_list) / 2:
                obl = True

        if range_list[0] < self.range_max:
            obmr = True
        if range_list[len(range_list) - 2] < self.range_max:
            obml = True

        if obr or obl or obmr or obml:
            rot = 0
            spd = 0

            if self.morse.rpc(morse_command, 'get_status') == 'Transit':
                self.morse.rpc(morse_command, 'stop')

            if (obmr or obml) and not obr and not obl:
                spd = 0.05
            elif not obmr and obl and ((not obr and not obml) or obr):
                rot = -0.1
            elif obr and not obl and (obmr or obml):
                rot = 0.1
                spd = 0.05
            elif obl and not obr and (obmr or obml):
                rot = -0.1
                spd = 0.05
            else:
                rot = 0.1

            self.morse.rpc('human' + str(self.human_id), 'move', spd, rot)
        elif self.morse.rpc(morse_command, 'get_status') == 'Stop':
                morse.rpc(morse_command, 'resume')

    def move(self,waypoints):
        self.last_10pos = [self.get_pose(self.human_id)] + [[0,0]] * 9
        wait = True 

        for waypoint in waypoints:
            # check whether there is a person in the next waypoint, if there is then wait.
            while wait:
                if self.total_human == 1:
                    break
                for i in range(self.total_human):
                    if i == self.human_id:
                        continue
                    other_pose = self.get_pose(i) 
                    if not self.is_near(waypoint[0], other_pose[0],
                            waypoint[1], other_pose[1], 1):
                        wait = False 
                        break
                    else:
                        time.sleep(0.5)

            morse_command = ['human' + str(self.human_id) + '.motion' +
                    str(self.human_id), 
                        'goto']
            #  Without Thread, morse.rpc for motion goto holds until the object
            #  arrives to the target position
            worker = threading.Thread(target=self.morse.rpc, args=(morse_command[0],
                    morse_command[1], waypoint[0], waypoint[1], 0.0,
                    self.waypoint_tolerance, self.waypoint_speed, ))
            worker.setDaemon(True)
            worker.start()
            time.sleep(0.1)

            while self.morse.rpc(morse_command[0], 'get_status') != 'Arrived':
                if random.randint(0,8) == 4:
                    self.last_10pos = self.last_10pos[1:] + \
                        [self.get_pose(self.human_id)]

                if self.stuck():
                    #self.morse.rpc('human' + str(self.human_id), 'move', 0, 3.2)
                    self.last_10pos = [self.get_pose(self.human_id)] + [[0,0]] * 9

                #self.obstacle_avoidance()
                time.sleep(0.1)

            worker.join()


# Get sequences of waypoints for each person. Sequences of waypoints for
# different persons are in different files.
def get_waypoints(PATH): 
    index = 0
    file_name = "wp" + str(index) + ".txt"

    # 4D dta, an element in the first level is sequences of waypoints for one
    # person. An element of the second level is a sequence of waypoints for
    # that particular person. An element on the third level is a waypoint for
    # the person.
    waypoints_of_waypoints = []

    while os.path.isfile(PATH + file_name) and os.access(PATH + file_name,
            os.R_OK):
        with open(PATH + file_name, "r") as fo:
            waypoints = []
            for line in fo:
                waypoint_sequence = []
                string_split = line.split()
                i = 0
                while i < len(string_split):
                    waypoint = [float(string_split[i]),
                            float(string_split[i+1])]
                    waypoint_sequence.append(waypoint)
                    i = i + 2
                waypoints.append(waypoint_sequence)
                waypoints.append(waypoint_sequence[::-1])
        waypoints_of_waypoints.append(waypoints)
        index = index + 1
        file_name = "wp" + str(index) + ".txt"

    return waypoints_of_waypoints

def get_wp_dict(PATH):
    if os.path.isfile(PATH + "wp_dict.yaml") and os.access(PATH +
            "wp_dict.yaml", os.R_OK):
        return yaml.load(open(PATH + "wp_dict.yaml"))

def get_move_plan(PATH):
    if os.path.isfile(PATH + "mv_plan.yaml") and os.access(PATH +
            "mv_plan.yaml", os.R_OK):
        return yaml.load(open(PATH + "mv_plan.yaml"))


if __name__ == "__main__":
    rospy.init_node('morse_humans_navigation') 
    wp_dict_folder = rospy.get_param("~wp_dict_folder")
    wp_folder = rospy.get_param("~waypoint_folder")

    waypoints_4d = get_waypoints(wp_folder)
    wp_dict = get_wp_dict(wp_dict_folder)
    mv_plan = get_move_plan(wp_dict_folder) 
    humans = []

    #if pymorse.Morse._asyncore_thread is not None:
    #    rospy.loginfo("[Human Controller] crikey")
    #    pymorse.Morse._asyncore_thread.join()
    #    pymorse.Morse._asyncore_thread = None
    try:
        rospy.loginfo("[Human Controller] is waiting for Morse...")
        with pymorse.Morse() as morse:
            for index in range(len(waypoints_4d)):
                human = Human(waypoints_4d[index], wp_dict[index], mv_plan[index], morse, index, len(waypoints_4d))
                humans.append(human)

            rospy.loginfo("[Human Controller] is running...")
            while True:
                time.sleep(0.002)
    except Exception as e:
        rospy.loginfo("[Human Controller] " + str(e) + " : failed.")
