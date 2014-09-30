#! /usr/bin/env python3.3 

import rospy
import yaml
from pymorse import Morse
from geometry_msgs.msg import Pose, PoseStamped

pose = Pose()

def pose_cb(data):
    global pose
    pose = data.pose

if __name__ == '__main__':
    rospy.init_node('waypoint_generator')
    wp_path = rospy.get_param("~wp_path")
    wp_dict_path = rospy.get_param("~wp_dict_path")
    
    rospy.Subscriber('/robot_pose', PoseStamped, pose_cb)
    
    with Morse() as morse:
        rospy.loginfo("[Waypoint Generator] is running...")
        print("""Press q to quit, r to record a waypoint, n to insert a new sequence of waypoints for the same person, and c to make waypoints or another person""")
        print("Every choice is followed by [ENTER]")
        print("Every 'r' can be followed by waypoint name, ex: r up right door")
        esc = False
        wp_index = 0
        counter = 0
        wp = dict()
        wp_dict = dict()
        wp_sequences = []
        wp_file = open(wp_path, "w")
        wp_dict_file = open(wp_dict_path, "w")

        while not esc:
            c = input()
            if c[0] == "r":
                if wp_index not in wp:
                    wp[wp_index] = dict() 
                    wp[wp_index][counter] = []
                    
                temp = [round(pose.position.x,2), round(pose.position.y,2)]
                wp_sequences.append(temp)
                
                if len(c) > 1:
                    wp_name = " ".join(c.split()[1:])
                    if wp_name != "" and wp_name not in wp_dict:
                        wp_dict[wp_name] = temp 
                    
            elif c[0] == "n":
                wp[wp_index][counter] = wp_sequences
                wp_sequences = []
                counter += 1
            elif c[0] == "q":
                wp[wp_index][counter] = wp_sequences
                esc = True
            elif c[0] == "c":
                wp[wp_index][counter] = wp_sequences
                wp_index += 1
                counter = 0
                wp_sequences = []

            rospy.sleep(0.2)

        wp_file.write(yaml.dump(wp))
        wp_dict_file.write(yaml.dump(wp_dict))
        wp_file.close()
        wp_dict_file.close()
