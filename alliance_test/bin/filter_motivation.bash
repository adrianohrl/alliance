#!/bin/bash

USAGE="Usage: filter_motivation [--bag | -b <bag file>]
                     [--help | -h] 
                     [--params | -p <robot params yaml file>]
                     [[--robot | -r <robot id>] ...]
                     [[--task | -t <task id>] ...]

  Author: Adriano Henrique Rossette Leite
  Version: 1.0.0
  Description: This script uses the rosbag filter tool in
               order to filter the motivation messages of a 
               specific robot according to its alliance 
               parameters YAML file.

  Options:

    -b, --bag <bag file>
         Informs the location of the desired motivation bag 
         file. (Default: '../bag/motivation.bag')
    -h, --help
         Shows this helper message for usage info.
    -p, --params
         Informs the robot's alliance parameters YAML file, 
         in order to take each task id. (Default: 
         ../config/<robot id>_alliance_params.yaml)
    -r, --robot <robot id>
         Informs the id of the desired robot.
    -t, --task <task id>
         Informs the id of the desired task.

"    

if [[ ${#} -lt 2 ]]; then
	echo "${USAGE}"
	exit
fi

INPUT_BAG_FILE="";
YAML_FILES=();
ROBOT_IDS=();
TASK_IDS=();
while [[ ${#} -gt 0 ]]; do
	case "${1}" in
	    -b|--bag)
		    INPUT_BAG_FILE="${2}"
		    shift
		    ;;
      -p|--params)
        YAML_FILE=(${2})
        shift
        ;;
      -r|--robot)
        ROBOT_IDS+=(${2})
        shift
        ;;
	    -t|--task)
		    TASK_IDS+=(${2})
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

if [[ -z ${INPUT_BAG_FILE} ]]; then
	INPUT_BAG_FILE="../bag/motivation.bag";
fi

if [[ ${INPUT_BAG_FILE} != *.bag ]]; then
	echo "ERROR: ${INPUT_BAG_FILE} must be a .bag file!!!"
	echo "Aborting..."
	exit 
fi

if [[ ${#ROBOT_IDS[@]} -eq 0 ]]; then
	echo "ERROR: Enter the id of the desired robot, please."
	echo "Aborting..."
	exit 
fi

echo "ROBOT_IDS         = ${ROBOT_IDS[@]}"
echo "TASK_IDS          = ${TASK_IDS[@]}"
echo "INPUT_BAG_FILE    = ${INPUT_BAG_FILE}"

TOPIC_NAME="/alliance/motivation";

if [[ ${#TASK_IDS[@]} -gt 0 ]]; then
	for ROBOT_ID in "${ROBOT_IDS[@]}"; do
		echo "ROBOT_ID          = ${ROBOT_ID}"
		echo "NUMBER_OF_TASKS   = ${#TASK_IDS[@]}"
		for TASK_ID in "${TASK_IDS[@]}"; do
			OUTPUT_BAG_FILE="../bag/${ROBOT_ID}-${TASK_ID}-motivation.bag";
			OUTPUT_CSV_FILE="../bag/${ROBOT_ID}-${TASK_ID}-motivation.csv";
			ROSBAG_EXPRESSION="m.header.frame_id.endswith('${ROBOT_ID}') and m.task_id == '${TASK_ID}'";
			OCTAVE_EXPRESSION="plot_motivation('${ROBOT_ID}', '${TASK_ID}', '${OUTPUT_CSV_FILE}')";

			echo "--------------------------------------------------------------------------------"
			echo "TASK_ID           = ${TASK_ID}"
			echo "OUTPUT_BAG_FILE   = ${OUTPUT_BAG_FILE}"
			echo "OUTPUT_CSV_FILE   = ${OUTPUT_CSV_FILE}"
			echo "ROSBAG_EXPRESSION = ${ROSBAG_EXPRESSION}"
			echo "OCTAVE_EXPRESSION = ${OCTAVE_EXPRESSION}"

			rosbag filter ${INPUT_BAG_FILE} ${OUTPUT_BAG_FILE} "${ROSBAG_EXPRESSION}"
			echo ${OUTPUT_CSV_FILE_HEADER} > ${OUTPUT_CSV_FILE}
			rostopic echo -b ${OUTPUT_BAG_FILE} -p ${TOPIC_NAME} >> ${OUTPUT_CSV_FILE}
			sed -i -e "${SED_PATTERN}" ${OUTPUT_CSV_FILE}
			if [[ $(which octave) ]]; then
    octave --silent --eval "${OCTAVE_EXPRESSION}"
				NEW_OUTPUT_CSV_FILE="../bag/${ROBOT_ID}-${TASK_ID}-motivation-new.csv";
				if [[ ! -f "${NEW_OUTPUT_CSV_FILE}" ]]; then
					echo "ERROR: No such file: ${NEW_OUTPUT_CSV_FILE}"
					continue
				fi
				LINK_DIRECTORY=~/Documents/mastering/dissertation/figures/motivation;
    LINK_NAME="${ROBOT_ID//\_/-}-${TASK_ID//\_/-}-motivation-new.csv";
				if [[ ! -d "${LINK_DIRECTORY}" ]]; then
					echo "ERROR: No such directory: ${LINK_DIRECTORY}"
					continue
    fi
				ln --force --symbolic $(readlink -f ${NEW_OUTPUT_CSV_FILE}) ${LINK_DIRECTORY}/${LINK_NAME}
			fi
		done
		echo "================================================================================"
	done
	echo "Exiting"
	exit
fi

if [[ ${YAML_FILES} -eq 0 ]]; then
	for ROBOT_ID in "${ROBOT_IDS[@]}"; do
		YAML_FILES+=("../config/${ROBOT_ID}_alliance_params.yaml")
	done
fi

if [[ "${#YAML_FILES[@]}" -ne "${#ROBOT_IDS[@]}" ]]; then
	echo "ERROR: Each input robot_id must have its own .yaml file if none task_id is given."
	echo "Aborting..."
	exit 
fi

if [[ ! $(which shyaml) ]]; then
	echo "ERROR: shyaml not found. Run the following command for that:"
	echo ""
	echo "   sudo pip install shyaml"
	echo ""
	echo "For more information: https://github.com/0k/shyaml"
	exit 
fi

for ((i = 0; i < ${#ROBOT_IDS[@]}; ++i)); do
	ROBOT_ID=${ROBOT_IDS[${i}]}
	YAML_FILE=${YAML_FILES[${i}]}

	if [[ ! -f ${YAML_FILE} ]]; then
		echo "ERROR: The ${ROBOT_ID}'s alliance parameters yaml file does not exist: ${YAML_FILE} ."
		echo "Aborting..."
		exit 
	fi

	NUMBER_OF_TASKS=$(cat ${YAML_FILE} | shyaml get-value behaviour_sets.size)
	echo "ROBOT_ID          = ${ROBOT_ID}"
	echo "YAML_FILE         = ${YAML_FILE}"
	echo "NUMBER_OF_TASKS   = ${NUMBER_OF_TASKS}"

	for ((j = 0; j < ${NUMBER_OF_TASKS}; ++j)); do
		TASK_ID=$(cat ${YAML_FILE} | shyaml get-value behaviour_sets.behaviour_set${j}.task_id)
		OUTPUT_BAG_FILE="../bag/${ROBOT_ID}-${TASK_ID}-motivation.bag";
		OUTPUT_CSV_FILE="../bag/${ROBOT_ID}-${TASK_ID}-motivation.csv";
		OUTPUT_CSV_FILE_HEADER="%time,impatience,acquiescent,suppressed,resetted,aplicable,motivation,threshold,active (for the ${ROBOT_ID}/${TASK_ID} behaviour set)";
		ROSBAG_EXPRESSION="m.header.frame_id.endswith('${ROBOT_ID}') and m.task_id == '${TASK_ID}'";
		SED_PATTERN="2d;s/.*,.*,\(.*\),.*${ROBOT_ID},${TASK_ID}/\1/g;";
		OCTAVE_EXPRESSION="plot_motivation('${ROBOT_ID}', '${TASK_ID}', '${OUTPUT_CSV_FILE}')";

		echo "--------------------------------------------------------------------------------"
		echo "j                 = ${j}"
		echo "TASK_ID           = ${TASK_ID}"
		echo "OUTPUT_BAG_FILE   = ${OUTPUT_BAG_FILE}"
		echo "OUTPUT_CSV_FILE   = ${OUTPUT_CSV_FILE}"
		echo "ROSBAG_EXPRESSION = ${ROSBAG_EXPRESSION}"
		echo "SED_PATTERN       = ${SED_PATTERN}"
		echo "OCTAVE_EXPRESSION = ${OCTAVE_EXPRESSION}"

		rosbag filter ${INPUT_BAG_FILE} ${OUTPUT_BAG_FILE} "${ROSBAG_EXPRESSION}"
		echo ${OUTPUT_CSV_FILE_HEADER} > ${OUTPUT_CSV_FILE}
		rostopic echo -b ${OUTPUT_BAG_FILE} -p ${TOPIC_NAME} >> ${OUTPUT_CSV_FILE}
		sed -i -e "${SED_PATTERN}" ${OUTPUT_CSV_FILE}
		if [[ $(which octave) ]]; then
   octave --silent --eval "${OCTAVE_EXPRESSION}"
			NEW_OUTPUT_CSV_FILE="../bag/${ROBOT_ID}-${TASK_ID}-motivation-new.csv";
			if [[ ! -f "${NEW_OUTPUT_CSV_FILE}" ]]; then
				echo "ERROR: No such file: ${NEW_OUTPUT_CSV_FILE}"
				continue
			fi
			LINK_DIRECTORY=~/Documents/mastering/dissertation/figures/motivation;
   LINK_NAME="${ROBOT_ID//\_/-}-${TASK_ID//\_/-}-motivation-new.csv";
			if [[ ! -d "${LINK_DIRECTORY}" ]]; then
				echo "ERROR: No such directory: ${LINK_DIRECTORY}"
				continue
   fi
			ln --force --symbolic $(readlink -f ${NEW_OUTPUT_CSV_FILE}) ${LINK_DIRECTORY}/${LINK_NAME}
		fi
	done
	echo "================================================================================"
done
