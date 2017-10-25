# run_mobilesim

This tool launches the simulator [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) for MRS applications.

## Usage:

    run_mobilesim [--help | -h] 
                  [--map | -m <map path>] 
                  [--num-robots | -n <number of robots>]
                  [--ros-pkg | -p <ROS package name>]

-------------------

# run_alliance_test_mobilesim

This tool simplifies the usage of the run_mobilesim tool for the ROS package alliance_test.

So, in order to use these tools, firstly enter the following command to a terminal:

	roscd alliance_test/bin
	BASH_FILE=../../../bin/run_mobilesim.bash
	chmod +x ${BASH_FILE} ./run_alliance_test_mobilesim.bash
	ln -s ${BASH_FILE} .

And then, just enter the following command:

	./run_alliance_test_mobilesim.bash map3

-------------------

# filter_motivation

This script uses the rosbag filter tool in order to filter the motivation messages of a specific robot according to its alliance parameters YAML file.

## Dependencies

This tool depends on [SHYAML](https://github.com/0k/shyaml) package. In order to install it, run:

	sudo pip install shyaml

## Setup

Run the following commands in a terminal to properly use this tool:

	roscd alliance_test/bin
	chmod +x ./filter_motivation.bash

## Usage

Run the alliance_test application. For example:

	roscd alliance_test/bin && ./run_alliance_test_mobilesim.bash map3

In another terminal:
	
	roslaunch alliance_test multiple_robots.launch

Note that this launch records the '/alliance/motivation' topic to a BAG file named motivation.bag at the bag folder of the alliance_test package. So given this recorded file, run the following commands to separate each .bag file correspond to each task that robot2 can perform (given by robot2_alliance_params.yaml file).

	roscd alliance_test/bin
	./filter_motivation.bash -r robot2

Verify that it was generated new .bag and .csv files at bag folder of alliance_test package:

	rosls alliance_test/bag | grep robot2