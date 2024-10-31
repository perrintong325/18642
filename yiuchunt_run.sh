#!/bin/bash

# Will kill all processes that are passed as arguments
# Used with trap to guarantee that CTRL+C on this script kills all BG processes
kill_processes() {
    for p in "$@"; do
	if [[ -z $(ps -p $p > /dev/null) ]]; then
	    echo "Killing process $p"
	    kill $p
	    sleep 1
	fi
    done
    echo "killed all processes, exiting"
    exit 0
}

# Arguments checking
if [[ $# -gt 1 ]]; then
    echo "Only one argument allowed: yiuchunt_run.sh [-h|--help] [MAZEFILE_NAME]"
    exit 1
fi

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: ./yiuchunt_run.sh [-h|--help] [MAZEFILE_NAME]"
    exit 2
fi

maze_number=${1:-1}
maze_file="m${maze_number}.maze"

# Run roscore if it is not already running
ROSCORE_PID=""
if [[ -z $(pgrep roscore) ]]; then
    roscore&
    ROSCORE_PID=$!
    sleep 1
    if [[ -z $(pgrep roscore) ]]; then
	echo "Error launching roscore. Make sure no other ros processes are running and try again."
	exit 1
    fi
fi
# Have to kill BG process if user exits
trap 'kill_processes $ROSCORE_PID' SIGINT
sleep 5

# If maze file argument is provided, pass it to the backend via ros param
# (code backend defaults to m1.maze so we don't worry about that case)
rosparam set /maze_file "$maze_file"

# Node that displays the maze and runs the turtle
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1
if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node. Have you run catkin_make?"
    kill_processes $ROSCORE_PID
    exit 1
fi
# Have to kill BG processes if user exits
trap 'kill_processes $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 9

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!
# Have to kill BG processes if user exits
trap 'kill_processes $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT

# Spin
while [ 1 -eq 1 ]; do
    sleep 30
done

