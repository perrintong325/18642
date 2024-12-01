#include "monitor_interface.h"

// Variables to track the turtle's state
bool maze_solved = false;
int last_x = -1;
int last_y = -1;
Orientation last_orientation = EAST;
bool orientation_initialized = false;

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (maze_solved) {
        // Check if the turtle has moved or turned after solving the maze
        if (x != last_x || y != last_y) {
            ROS_WARN("VIOLATION: Turtle moved after solving the maze!");
        }
        if (o != last_orientation) {
            ROS_WARN("VIOLATION: Turtle turned after solving the maze!");
        }
    }

    // Update the last known position and orientation
    last_x = x;
    last_y = y;
    last_orientation = o;
    orientation_initialized = true;
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (atEnd) {
        maze_solved = true;
    }
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}