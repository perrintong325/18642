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
maze_file=""
if [[ $# -gt 0 ]]; then
    echo "No arguments allowed: ./yiuchunt_build.sh"
    exit 1
fi

target_directory="catkin_ws/src"

if [ -d "$target_directory/ece642rtle" ]; then
    echo "Removing existing ece642rtle directory"
    rm -rf "$target_directory/ece642rtle"
fi

tar -zxvf yiuchunt_files.tgz -C "$target_directory"

# Locate the ece642rtle directory
turtledir="$target_directory/ece642rtle"

# Catkin WS is two directories above package directory
cd "$turtledir/../.."
echo "Working from catkin workspace $(pwd)"
echo ""
catkin_make -DCATKIN_WHITELIST_PACKAGES="ece642rtle"
# Build the student node
catkin_make ece642rtle_student
if [ $? -ne 0 ]; then
    echo "catkin_make did not succeed, exiting script"
    exit 1
fi

# So ROS can find everything
source devel/setup.bash

cd ~

echo "Build successful"
