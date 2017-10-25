#!/bin/bash

USAGE="Usage: run_alliance_test_mobilesim [-h | --help] [<map name> [<number of robots>]]

  Author: Adriano Henrique Rossette Leite
  Version: 1.0.0
  Description: This script runs multiple Adept Pioneer 3 DX 
               robots in the MobileSim simulator for Multi 
               Robot System (MRS) application simulations.
"

MAP_PATH=""
if [[ ${#} > 0 && ${1} == "-h" || ${1} == "--help" ]]; then
	echo "${USAGE}"
	exit
fi

MAP_PATH="map/map3.map"
if [[ ${#} -gt 0 ]]; then
  MAP_PATH="map/${1}.map"
fi
ROS_PKG="alliance_test"
NUMBER_ROBOTS=5
if [[ ${#} -gt 1 ]]; then
  NUMBER_ROBOTS=${2}
fi

./run_mobilesim.bash -m ${MAP_PATH} -p ${ROS_PKG} -n ${NUMBER_ROBOTS}
