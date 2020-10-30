#!/bin/bash
#
# Run rosbag play, visualization and save trajectory in a text file.
# It requires a catkin workspace containing pose_listener.

set -eE # Any subsequent commands which fail will cause the shell script to exit immediately
OUTPUT_TOPIC="/vins_estimator/odometry"
CATKIN_WS_DIR=$HOME/catkin_ws/

function echoUsage()
{
    echo -e "Usage: ./run_rosario_sequence.sh [FLAG] \n\
            \t -r run method in a detached docker container \n\
            \t -o path to output file \n\
            \t -h help" >&2
}

RUN_CONTAINER=0
OUTPUT_FILE="trajectory.txt"
while getopts "hro:" opt; do
    case "$opt" in
        h)
            echoUsage
            exit 0
            ;;
        r)  RUN_CONTAINER=1
            ;;
        o)  case $OPTARG in
                -*) echo "ERROR: a path to output file must be provided"; echoUsage; exit 1 ;;
                *) OUTPUT_FILE=$OPTARG ;;
            esac
            ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

shift $((OPTIND -1))
BAG=$1

function cleanup() {
  printf "\e[31m%s %s\e[m\n" "Cleaning"
  if [ -n "${CID}" ] ; then
    docker container stop $CID
    # unset CID
  fi
  # rosnode kill -a
}

trap cleanup INT
trap cleanup ERR

function wait_docker() {
    TOPIC=$1
    output=$(rostopic list $TOPIC 2> /dev/null || :) # force the command to exit successfully (i.e. $? == 0)
    echo $output
    while [ "$output" != $TOPIC ]; do # TODO max attempts
        sleep 1
        output=$(rostopic list $TOPIC 2> /dev/null || :)
    done
}

if [ $RUN_CONTAINER -eq 1 ] ; then
    echo "Starting docker container (detached mode)"
    CID=$(./run_vins_fusion.sh -v detached)
fi

wait_docker $OUTPUT_TOPIC

source ${CATKIN_WS_DIR}/devel/setup.bash
ROS_HOME=`pwd` roslaunch launch/play_bag_viz.launch \
    config_rviz:=$(pwd)/config/vins_rviz_config.rviz \
    type:=O \
    topic:=$OUTPUT_TOPIC \
    save_to_file:=true \
    output_file:=$OUTPUT_FILE \
    bagfile:=$BAG

#docker container logs $CID > logs_$CID.txt
cleanup
echo "END"