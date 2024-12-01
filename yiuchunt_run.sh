#!/bin/bash

# Will kill all processes that are passed as arguments
# Used with trap to guarantee that CTRL+C on this script kills all BG processes
kill_processes() {
    touch VIOLATIONS.txt
    rm VIOLATIONS.txt
    total_viol=0
    while [[ $# -gt 0 ]]; do
        m=$1
        shift
        if [[ -n $(pgrep -x ${m:0:15}) ]]; then
            echo "Killing monitor $m"
            kill $(pgrep -x ${m:0:15})
            sleep 2
        fi
        # Process any non-empty monitor file
        if [ -s "$m.output.tmp" ]; then
            echo "" >> VIOLATIONS.txt
            echo "Monitor $m Violations:" >> VIOLATIONS.txt
            echo "" >> VIOLATIONS.txt
            grep -C 5 "[ WARN]" $m.output.tmp >> VIOLATIONS.txt
            m_viol=$(grep "[ WARN]" $m.output.tmp | wc -l)
            total_viol=$(( total_viol + m_viol ))
            rm $m.output.tmp
            echo "" >> VIOLATIONS.txt
        fi
    done
    echo "TOTAL VIOLATIONS: $total_viol"
    echo "Any violations logged in VIOLATIONS.txt"
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
target_directory="catkin_ws"
cd "$target_directory"
echo "Working from catkin workspace $(pwd)"
echo ""
source devel/setup.bash

# Start the monitors and capture their PIDs
MONITORS=(
    "ece642rtle_logging_monitor"
    "ece642rtle_step_monitor"
    "ece642rtle_turn_monitor"
    "ece642rtle_atend_monitor"
    "ece642rtle_face_monitor"
    "ece642rtle_forward_monitor"
    "ece642rtle_solved_monitor"
    "ece642rtle_tick_monitor"
    "ece642rtle_wall_monitor"
)

for mon in "${MONITORS[@]}"; do
    echo "Starting monitor $mon"
    stdbuf -oL rosrun ece642rtle $mon | tee $mon.output.tmp &
    sleep 1
    # Need -f for full name because process name might be long
    if [[ -z $(pgrep ${mon:0:15}) ]]; then
        echo "Error launching $mon. Have you run catkin_make?"
        rm $mon.output.tmp
        kill_processes "${MONITORS[@]}"
        exit 1
    fi
done

rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1
if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node. Have you run catkin_make?"
    kill_processes $ROSCORE_PID
    exit 1
fi
# Have to kill BG processes if user exits
trap 'kill_processes "${MONITORS[@]}" $TURTLE_PID' SIGINT
sleep 9

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!
# Have to kill BG processes if user exits
trap 'kill_processes "${MONITORS[@]}" $TURTLE_PID $STUDENT_PID' SIGINT

# Spin
while [ 1 -eq 1 ]; do
    sleep 30
done

