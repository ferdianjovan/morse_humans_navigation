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

To start this human_following module, simplly launch:
    
    roslaunch morse_humans_navigation humans_navigation.launch
    
Configurations
==========================
To set up new behaviours for robots regarding their movements in the simulation, 
    
    
Notes
============================

All the robots in the simulation must be named in the form
    
    human[number]

and all waypoint class attached to the robots must be named in the form
    
    motion[number]

where [number] starts from 0 to the number of robots in the simulation minus one.
