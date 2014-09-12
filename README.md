morse_humans_navigation
=======================

multiple robots/humans navigation using waypoints

Run:
  roslaunch morse_humans_navigation humans_navigation.launch
  
You need to run morse environment first with each a robot named "human[somenumber]" possessing pose information named "pose[somenumber]" in your simulator. It currently works perfectly with https://github.com/ferdianjovan/strands_morse

To create your own waypoints for each person, you need to run
   roslaunch morse_humans_navigation waypoint_generator.launch

Follow the instruction in the terminal where you run the waypoint_generator.launch
