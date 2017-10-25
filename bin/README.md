# Application Simulation

These tools launch the simulator [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) for MRS applications.

Usage:

    run_mobilesim [--help | -h] 
                  [--map | -m <map path>] 
                  [--num-robots | -n <number of robots>]
                  [--ros-pkg | -p <ROS package name>]

-------------------

# run_mobilesim

This script runs multiple Adept Pioneer 3 DX robots in the MobileSim simulator for Multi Robot System (MRS) application simulations.

So, in order to use these tools, firstly enter the following command to a terminal:

	chmod +x ./run_mobilesim.bash

Firstly, check if the /usr/local/MobileSim/PioneerRobotModels.world.inc file contains the following definitions:

	# This is a quick hack to have several differently 
	# colored Pioneers in the same simulation:
	define red_p3dx p3dx (color "red")
	define black_p3dx p3dx (color "black")
	define blue_p3dx p3dx (color "blue")
	define violet_p3dx p3dx (color "violet")
	define green_p3dx p3dx (color "green")
	define orange_p3dx p3dx (color "orange")
	define magenta_p3dx p3dx (color "magenta")
	define cyan_p3dx p3dx (color "cyan")
	define purple_p3dx p3dx (color "purple")
	define yellow_p3dx p3dx (color "yellow")
	define pink_p3dx p3dx (color "pink")

-------------------

# Creating maps

Use the application [Mapper3-Basic](http://robots.mobilerobots.com/wiki/Mapper3Basic) to create/edit maps in order to define the space to simulate in the simulator [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) according to the desired application.

-------------------

# Clang Format

Firstly, make sure you have [clang-format-3.9](http://releases.llvm.org/3.4/tools/clang/docs/ClangFormatStyleOptions.html) installed. 

	sudo apt-get install clang-format-3.9

Secondly, in order to use these tools, enter the following command to a terminal:

	chmod +x ./clang-format.bash

Finally, enter the following command in order to format all files all in once:

	./clang-format.bash
