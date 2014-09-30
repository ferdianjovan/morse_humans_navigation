morse_humans_navigation
=======================

This package contains human navigation package run in MORSE simulator. 
The main function of this package is to navigate waypoint class attached to robot class in MORSE based on simple Markov chain defined in mv_plan.yaml.
The other function is to provide sequences of waypoints for each waypoint class attached to a robot (stored in wp.yaml) and the name of particular waypoints if it is necessary (stored in wp_dict.yaml).
Particular waypoints are needed to be defined to construct the simple Markov chain defined in mv_plan.yaml. 

Getting Start
=========================

Before you can launch this package, you need:
    
    strands_morse simulation up and running
    
One should note that the number of robots with waypoint class attached to them must match with the number of Markov chains defined in mv_plan.yaml and the number of sets of sequences of waypoints defined in wp.yaml.

To start this human_following module, simply launch:
    
    roslaunch morse_humans_navigation humans_navigation.launch
    
Configurations
==========================
To set up new behaviours for robots regarding their movements in the simulation, you need:

    strands_morse simulation up and running
    
To start constructing waypoints, launch:

    roslaunch morse_humans_navigation waypoint_generator.launch
    
Please add Pose class to Scitosa5 robot described in Notes section. Other instructions are stated in the window running the waypoint generator launch file.
    
    
Notes
============================

All the robots in the simulation must be named in the form
    
    human[number]

and all waypoint and pose class attached to the robots must be named in the form
    
    motion[number]
    pose[number]
    
where [number] starts from 0 to the number of robots in the simulation minus one.
For example,
 
    human0 = B21()
    human0.translate(x=-14.28,-9.28,z=0.1)
    human0.rotate(z=1.6)
    pose0 = Pose()
    human0.append(pose0)
    motion0 = Waypoint() 
    motion0.properties(ControlType="Position",ObstacleAvoidance=True)
    motion0.add_service('socket')
    human0.append(motion0)
    pose0.add_service('socket')
    pose0.add_interface('ros')
    human0.properties(Object = True, Label = "human0")

Also add Pose class to Scitosa5 robot in the simulation as follows: 

    pose = Pose()
    robot.append(pose)
    pose.add_interface('ros', topic='/robot_pose')
    
