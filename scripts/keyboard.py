#! /usr/bin/env python3.3

import rospy
import sys
import threading
from pymorse import Morse

if __name__ == '__main__':
    rospy.init_node('human0_controller')
    rospy.loginfo("Use WASD to control human0")
    with Morse() as simu:
        simu.rpc('simulation', 'deactivate', 'human0.motion0')
        rospy.sleep(0.5)
        v = 0.0
        w = 0.0

        while not rospy.is_shutdown():
            key = input()
            if key.lower() == "w":
                v += 0.5
            elif key.lower() == "s":
                v -= 0.5
            elif key.lower() == "a":
                w += 0.3
            elif key.lower() == "d":
                w -= 0.3
            else:
                continue

            worker = threading.Thread(target=simu.rpc, args=('human0.second_motion', 'set_speed', v, w, ))
            worker.setDaemon(True)
            worker.start()
            rospy.sleep(0.01)
