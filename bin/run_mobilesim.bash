#!/bin/bash

ROBOT_COLORS=(
	"red" 
	"black" 
	"blue" 
	"violet" 
	"green" 
	"orange" 
	"magenta" 
	"cyan" 
	"purple" 
	"yellow" 
	"pink"
	)
USAGE="Usage: run_mobilesim [--help | -h] 
                     [--map | -m <map path>] 
                     [--num-robots | -n <number of robots>]
                     [--ros-pkg | -p <ROS package name>]

  Author: Adriano Henrique Rossette Leite
  Version: 1.0.0
  Description: This script runs multiple Adept Pioneer 3 DX 
               robots in the MobileSim simulator for Multi 
               Robot System (MRS) application simulations.

  Options:

    -h, --help
         Shows this helper message for usage info.
    -m, --map <map path>
         Informs the relative path of the application map
         from the given ROS package.
    -n, --num-robots <repository>
         Informs the desired number of robots to be 
         launched in the simulator. The default value is 
         1 robots. The maximum number of different colors 
         is ${#ROBOT_COLORS[@]}.
    -p, --ros-pkg <ROS package name>
         Informs the ROS package name in which the 
         application map is located


  Notes:

    Check if the /usr/local/MobileSim/PioneerRobotModels.world.inc 
    file contains the following definitions:

         # This is a quick hack to have several differently 
         # colored Pioneers in the same simulation:
"    

for ROBOT_COLOR in ${ROBOT_COLORS[@]}; do
	USAGE+="         define ${ROBOT_COLOR}_p3dx p3dx (color \"${ROBOT_COLOR}\")"$'\n'
done

if [[ ${#} = 0 ]]; then
	echo "${USAGE}"
	exit
fi

MAP_PATH="";
NUMBER_OF_ROBOTS=2;
PACKAGE_NAME="";
while [[ ${#} -gt 0 ]]; do
	case "${1}" in
	    -m|--map)
		    MAP_PATH="${2}"
		    shift
		    ;;
	    -n|--num-robots)
		    NUMBER_OF_ROBOTS="${2}"
		    shift
		    ;;
	    -p|--ros-pkg)
		    PACKAGE_NAME="${2}"
		    shift
		    ;;
	    -h|--help)
			echo "${USAGE}"
			exit
		    ;;
	    *)
		    ;;
	esac
	shift
done

if [[ -z ${MAP_PATH} ]]; then
	echo "ERROR: Enter the map relative path, please."
    echo "Aborting..."
    exit 
fi

if [[ ${MAP_PATH} != *.map ]]; then
	echo "ERROR: ${MAP_PATH} must be a .map file!!!"
    echo "Aborting..."
    exit 
fi

if [[ -z ${PACKAGE_NAME} ]]; then
	echo "ERROR: Enter the ROS package name, please."
    echo "Aborting..."
    exit 
fi

PACKAGE_PATH=$(rospack find ${PACKAGE_NAME})

if [[ -z ${PACKAGE_PATH} ]]; then
    echo "Aborting..."
    exit 
fi

MAP_PATH=${PACKAGE_PATH}/${MAP_PATH}

if [[ ! -f ${MAP_PATH} ]]; then
    echo "ERROR: ${MAP_PATH} file not found!!!"
    echo "Aborting..."
    exit 
fi

if [[ ! ${NUMBER_OF_ROBOTS} =~ ^[-+]?([1-9][[:digit:]]*|0)$ || ${NUMBER_OF_ROBOTS} -lt 1 || ${NUMBER_OF_ROBOTS} -gt ${#ROBOT_COLORS[@]} ]]; then
	echo "ERROR: The desired number of robots must be a positive integer within the [1; ${#ROBOT_COLORS[@]}] interval."
    echo "Aborting..."
    exit 
fi

echo "PACKAGE_PATH     = ${PACKAGE_PATH}"
echo "MAP_PATH         = ${MAP_PATH}"
echo "NUMBER_OF_ROBOTS = ${NUMBER_OF_ROBOTS}"

ROBOTS=""
for ((i = 0; i < ${NUMBER_OF_ROBOTS}; ++i)); do
	ROBOTS+="-r ${ROBOT_COLORS[${i}]}_p3dx:p3dx${i} "
done;

MobileSim --map ${MAP_PATH} ${ROBOTS}