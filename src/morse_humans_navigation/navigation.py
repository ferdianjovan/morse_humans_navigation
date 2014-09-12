#! /usr/bin/env python3.3 

import sys
import rospy
import pymorse
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
        self.completing_waypoints()
        self.last_10pos = []
        self.range_max = 0.6
        self.waypoint_speed = 1 
        self.waypoint_tolerance = 0.5
        worker = threading.Thread(target=self.move_planning)
        worker.setDaemon(True)
        worker.start()
    
    def completing_waypoints(self):
        wp_seqs = []
        reverse_wp_seqs = []
        for i in self.waypoint_sequences.values():
            wp_seqs.append(i)
            reverse_wp_seqs.append(i[::-1])
        self.waypoint_sequences = wp_seqs + reverse_wp_seqs
        
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
                while waypoints == []:
                    for i in self.waypoint_sequences:
                        init_wp_flag = False
                        for j in i:
                            if self.is_near(pose[0],j[0],pose[1],j[1],wp_threshold):
                                init_wp_flag = True
                                continue
                            if init_wp_flag and self.is_near(next_wp[0],j[0],
                                    next_wp[1],j[1],wp_threshold):
                                waypoints = i
                                break
                        if waypoints != []:
                            break
                                    
                    wp_threshold += 0.1
                    
                self.move(waypoints)
    
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
                        rospy.sleep(0.5)

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
            rospy.sleep(0.1)

            while self.morse.rpc(morse_command[0], 'get_status') != 'Arrived':
                if random.randint(0,8) == 4:
                    self.last_10pos = self.last_10pos[1:] + \
                        [self.get_pose(self.human_id)]

                if self.stuck():
                    #self.morse.rpc('human' + str(self.human_id), 'move', 0, 3.2)
                    self.last_10pos = [self.get_pose(self.human_id)] + [[0,0]] * 9

                #self.obstacle_avoidance()
                rospy.sleep(0.1)

            worker.join()


if __name__ == "__main__":
    rospy.init_node('morse_humans_navigation') 

    waypoints_4d = yaml.load(open(rospy.get_param("~wp_path")))
    wp_dict = yaml.load(open(rospy.get_param("~wp_dict_path")))
    mv_plan = yaml.load(open(rospy.get_param("~mv_plan_path")))
    human_keyboard = rospy.get_param("~human_keyboard") 
    humans = []
    
    if len(waypoints_4d) != len(mv_plan):
        rospy.loginfo("The number of persons represented by movement planning and waypoints does not match! [Human Controller] will exit...")
        sys.exit()

    try:
        rospy.loginfo("[Human Controller] is waiting for Morse...")
        with pymorse.Morse() as morse:
            # deactivate human0's keyboard
            morse.rpc('simulation', 'deactivate', human_keyboard)

            for index in range(len(waypoints_4d)):
                human = Human(waypoints_4d[index], wp_dict, mv_plan[index], morse, index, len(waypoints_4d))
                humans.append(human)

            rospy.loginfo("[Human Controller] is running...")
            while True:
                rospy.sleep(0.002)
    except Exception as e:
        rospy.loginfo("[Human Controller] " + str(e) + " : failed.")
