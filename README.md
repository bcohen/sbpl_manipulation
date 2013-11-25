sbpl_manipulation
=================

This repository contains a set of motion planners for robotic arms that use the SBPL planning library.

1) Get the code:

	git clone https://github.com/bcohen/sbpl_manipulation

	git clone -b groovy https://github.com/bcohen/leatherman

	git clone https://github.com/sbpl/sbpl_geometry_utils

	git clone https://github.com/sbpl/sbpl

2) Build everything:

	rosmake sbpl_arm_planner_test

3) Bring up a roscore & rviz (I do it seperatly, you don't have to)

	Terminal 1: roscore
	Terminal 2: rosrun rviz rviz

	In rviz:
		a) Set Fixed Frame: base_footprint
		b) Add display for visualization_markers

3a) This is a temporary hack required to deal with a temporarily hardcoded path:

	roscd sbpl_arm_planner
	cp config/pr2.mprim /tmp

4) Plan for the PR2:

	roslaunch sbpl_arm_planner_test goal_pr2.launch

	If the plan was successful, you should see:

		1) Collision model of the robot
		2) Trajectory of the arm
		3) Tabletop (arm is moving around)
		4) Bounds of the environment

