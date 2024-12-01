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
    cd ~
    exit 0
}

# Arguments checking
if [[ $# -gt 0 ]]; then
    echo "No arguments allowed: ./yiuchunt_build_run_tests.sh"
    exit 1
fi

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: ./yiuchunt_build_run_tests.sh [-h|--help]"
    exit 2
fi

target_directory="catkin_ws/src/ece642rtle/student"
cd "$target_directory"

BUILD_DIR="build"
SRC_FILES="yiuchunt_student_test.cpp yiuchunt_student_turtle.cpp yiuchunt_mock_functions.cpp"
OUTPUT_FILE="student_tests"
CXX_FLAGS="-Dtesting -std=gnu++11"
LIBS="-lcunit"

# Compile the source files
g++ $CXX_FLAGS -o $OUTPUT_FILE $SRC_FILES $LIBS

# Check if the compilation was successful
if [[ $? -eq 0 ]]; then
    echo "Build successful. Executable created: $BUILD_DIR/$OUTPUT_FILE"
    # Run the tests
    ./$OUTPUT_FILE
else
    echo "Build failed."
fi

cd ~